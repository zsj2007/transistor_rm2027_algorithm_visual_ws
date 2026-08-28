#include "EKF/SuperPowerPredictor.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {
// 项目层与 SP 内部单位/角度转换所需常量。
constexpr double kMillimetersPerMeter = 1000.0;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kHalfPi = kPi / 2.0;
}

SuperPowerPredictor::SuperPowerPredictor(
    const EKFTargetObservation& initial_observation,
    double initial_radius_mm,
    std::shared_ptr<YAML::Node> config_file_ptr) {
    // 不继承旧估计器调参；以下默认值来自 SP standard3 与普通四装甲 set_target 分支。
    config_.min_detect_count = 5;
    config_.max_temp_lost_count = 15;
    config_.max_dt_s = 0.1;
    config_.initial_radius_m = 0.2;
    config_.armor_num = 4;

    // YAML 块只覆盖这些 SP 常量，集中管理配置来源，且不读取旧估计器配置。
    if (config_file_ptr) {
        const YAML::Node sp = (*config_file_ptr)["superpower_ekf"];
        if (sp) {
            if (sp["min_detect_count"])
                config_.min_detect_count = sp["min_detect_count"].as<int>();
            if (sp["max_temp_lost_count"])
                config_.max_temp_lost_count = sp["max_temp_lost_count"].as<int>();
            if (sp["max_dt_s"])
                config_.max_dt_s = sp["max_dt_s"].as<double>();
            if (sp["initial_radius_m"])
                config_.initial_radius_m = sp["initial_radius_m"].as<double>();
            if (sp["armor_num"])
                config_.armor_num = sp["armor_num"].as<int>();
            const YAML::Node angular_fit = sp["angular_velocity_fit"];
            if (angular_fit) {
                if (angular_fit["window_s"])
                    angular_velocity_fit_window_s_ = std::max(
                        0.02, angular_fit["window_s"].as<double>());
                if (angular_fit["min_samples"])
                    angular_velocity_fit_min_samples_ =
                        static_cast<std::size_t>(std::max(
                            2, angular_fit["min_samples"].as<int>()));
            }
            const YAML::Node joint = sp["joint_update"];
            if (joint) {
                if (joint["enabled"])
                    config_.pair_update.enabled =
                        joint["enabled"].as<bool>();
                if (joint["max_joint_nis"])
                    config_.pair_update.max_joint_nis =
                        joint["max_joint_nis"].as<double>();
                if (joint["max_secondary_position_error_m"])
                    config_.pair_update.max_secondary_position_error_m =
                        joint["max_secondary_position_error_m"].as<double>();
                if (joint["max_secondary_angle_error_rad"])
                    config_.pair_update.max_secondary_angle_error_rad =
                        joint["max_secondary_angle_error_rad"].as<double>();
                if (joint["measurement_variance_scale"])
                    config_.pair_update.measurement_variance_scale =
                        joint["measurement_variance_scale"].as<double>();
                if (joint["angle_variance_scale"])
                    config_.pair_update.angle_variance_scale =
                        joint["angle_variance_scale"].as<double>();
            }
        }
    }

    // 保持旧构造函数 ABI。SP 普通分支固定以 0.2 m 初始化，调用方旧 RMM 半径
    // 不得改变该基线。
    (void)initial_radius_mm;

    resetTracker();
    if (std::isfinite(initial_observation.t)) {
        initializeFromObservation(initial_observation);
    } else {
        last_dt_s_ = initial_observation.t;
        warnTimeIssue("non-finite timestamp", initial_observation.t,
                      initial_observation.t);
    }
}

void SuperPowerPredictor::update(
    const EKFTargetObservation& observation) {
    updateImpl(observation, std::nullopt);
}

void SuperPowerPredictor::updatePair(
    const EKFTargetObservation& primary,
    const EKFTargetObservation& secondary) {
    updateImpl(primary, secondary);
}

void SuperPowerPredictor::updateImpl(
    const EKFTargetObservation& primary,
    const std::optional<EKFTargetObservation>& secondary) {
    time_discontinuity_ = false;
    if (!std::isfinite(primary.t)) {
        last_dt_s_ = primary.t;
        warnTimeIssue("non-finite timestamp", primary.t, primary.t);
        return;
    }

    if (!has_update_time_) {
        initializeFromObservation(primary);
        timestamp_warning_active_ = false;
        return;
    }

    const double dt = primary.t - last_update_time_;
    last_dt_s_ = dt;
    if (!std::isfinite(dt)) {
        warnTimeIssue("non-finite dt", primary.t, dt);
        return;
    }
    if (dt <= 0.0) {
        warnTimeIssue("duplicate/out-of-order timestamp", primary.t, dt);
        return;
    }

    if (dt > config_.max_dt_s) {
        time_discontinuity_ = true;
        resetAngularVelocityFit();
    }

    last_observation_ = toSuperPower(primary);
    const bool secondary_valid =
        secondary.has_value() &&
        std::isfinite(secondary->t) &&
        std::abs(secondary->t - primary.t) <= 1e-6;
    if (secondary_valid && config_.pair_update.enabled) {
        const sp_ekf::ArmorObservation secondary_sp =
            toSuperPower(*secondary);
        last_result_ =
            tracker_->processPair(*last_observation_, secondary_sp, dt);
    } else {
        last_result_ = tracker_->process(last_observation_, dt);
        if (secondary && !secondary_valid) {
            last_result_.pair_requested = true;
            last_result_.pair_status = "TIMESTAMP_MISMATCH";
        }
    }
    last_update_time_ = primary.t;
    timestamp_warning_active_ = false;

    if (last_result_.initialized_this_frame) {
        update_frames_ = 1;
    } else if (last_result_.updated) {
        ++update_frames_;
    }
    if (last_result_.matched_id >= 0) {
        debug_flip_flag_ = (last_result_.matched_id % 2) + 1;
    }
    if (last_result_.initialized_this_frame) {
        resetAngularVelocityFit();
    }
    if (last_result_.updated && last_result_.matched_id >= 0) {
        observeAngularVelocity(primary.t);
    }
}

void SuperPowerPredictor::missUpdate(double update_time) {
    time_discontinuity_ = false;
    if (!std::isfinite(update_time)) {
        last_dt_s_ = update_time;
        warnTimeIssue("non-finite timestamp", update_time, update_time);
        return;
    }

    // 尚未收到任何观测时，空帧只能建立时间基准，不能创建 Target。
    if (!has_update_time_) {
        last_update_time_ = update_time;
        last_dt_s_ = 0.0;
        has_update_time_ = true;
        timestamp_warning_active_ = false;
        return;
    }

    const double dt = update_time - last_update_time_;
    last_dt_s_ = dt;
    if (!std::isfinite(dt)) {
        warnTimeIssue("non-finite dt", update_time, dt);
        return;
    }
    if (dt <= 0.0) {
        warnTimeIssue("duplicate/out-of-order timestamp", update_time, dt);
        return;
    }

    if (dt > config_.max_dt_s) {
        time_discontinuity_ = true;
        resetAngularVelocityFit();
    }

    last_observation_.reset();
    last_result_ = tracker_->process(std::nullopt, dt);
    last_update_time_ = update_time;
    timestamp_warning_active_ = false;
}

void SuperPowerPredictor::clear() {
    resetTracker();
    last_observation_.reset();
    last_update_time_ = 0.0;
    last_dt_s_ = 0.0;
    update_frames_ = 0;
    debug_flip_flag_ = 1;
    has_update_time_ = false;
    timestamp_warning_active_ = false;
    time_discontinuity_ = false;
}

EKFTargetPrediction SuperPowerPredictor::predict(double predict_time) const {
    EKFTargetPrediction result;
    if (!hasState()) return result;

    const sp_ekf::Target* target = tracker_->target();
    if (!target) return result;

    // 不修改滤波器后验状态；复制后按匀速/匀角速模型进行纯前向外推。
    Eigen::VectorXd x = target->ekfX();
    x[0] += x[1] * predict_time;
    x[2] += x[3] * predict_time;
    x[4] += x[5] * predict_time;
    x[6] = wrapAngle(x[6] + x[7] * predict_time);

    const double r1 = x[8];
    const double r2 = x[8] + x[9];
    const double h = x[10];

    result.center_x = x[0] * kMillimetersPerMeter;
    result.center_y = x[2] * kMillimetersPerMeter;
    result.center_z = x[4] * kMillimetersPerMeter;
    result.alternate_z = (x[4] + h) * kMillimetersPerMeter;
    result.r1 = r1 * kMillimetersPerMeter;
    result.r2 = r2 * kMillimetersPerMeter;
    result.h = h * kMillimetersPerMeter;
    result.yaw = toProjectYaw(x[6]);
    result.w = x[7];
    result.rotation_direction = x[7] >= 0.0 ? 1 : -1;

    // 由外推后的中心、相位和交替 r/l/h 几何重建每块装甲。
    const int armor_num = target->armorNum();
    result.armors.reserve(static_cast<std::size_t>(armor_num));
    for (int id = 0; id < armor_num; ++id) {
        const double angle = wrapAngle(
            x[6] + id * 2.0 * kPi / static_cast<double>(armor_num));
        const bool use_l_h = armor_num == 4 && (id == 1 || id == 3);
        const double radius = use_l_h ? r2 : r1;
        const double armor_x = x[0] - radius * std::cos(angle);
        const double armor_y = x[2] - radius * std::sin(angle);
        const double armor_z = use_l_h ? x[4] + h : x[4];

        result.armors.push_back(EKFPredictedArmor{
            armor_x * kMillimetersPerMeter,
            armor_y * kMillimetersPerMeter,
            armor_z * kMillimetersPerMeter,
            radius * kMillimetersPerMeter,
            toProjectYaw(angle),
        });
    }

    return result;
}

EKFTargetState SuperPowerPredictor::state() const {
    EKFTargetState result;
    if (!hasState()) return result;

    const sp_ekf::Target* target = tracker_->target();
    if (!target) return result;

    // 该接口导出当前后验，不包含 predict() 的未来外推。
    const Eigen::VectorXd x = target->ekfX();
    result.center_x = x[0] * kMillimetersPerMeter;
    result.center_vx = x[1] * kMillimetersPerMeter;
    result.center_y = x[2] * kMillimetersPerMeter;
    result.center_vy = x[3] * kMillimetersPerMeter;
    result.center_z = x[4] * kMillimetersPerMeter;
    result.center_vz = x[5] * kMillimetersPerMeter;
    result.yaw = toProjectYaw(x[6]);
    result.w = x[7];
    result.r1 = x[8] * kMillimetersPerMeter;
    result.r2 = (x[8] + x[9]) * kMillimetersPerMeter;
    result.h = x[10] * kMillimetersPerMeter;
    result.update_frames = update_frames_;
    return result;
}

EKFTargetDebugState SuperPowerPredictor::debugState() const {
    // 仅镜像 Tracker/EKF 的已有诊断值
    EKFTargetDebugState debug;
    debug.dt_s = last_dt_s_;
    debug.time_discontinuity = time_discontinuity_;
    debug.tracker_state = sp_ekf::trackerStateName(last_result_.state);
    debug.tracker_state_before =
        sp_ekf::trackerStateName(last_result_.state_before);
    debug.matched_id = last_result_.matched_id;
    debug.current_armor_id = last_result_.matched_id;
    debug.best_id = last_result_.matched_id;
    debug.measurement_valid = last_result_.measurement_valid;
    debug.updated = last_result_.updated;
    debug.lost_frames = last_result_.lost_frames;
    debug.nis = last_result_.nis;
    debug.position_error_m = last_result_.position_error;
    debug.yaw_error_deg = last_result_.angle_error >= 0.0
                              ? last_result_.angle_error * 180.0 / kPi
                              : -1.0;
    debug.phase_observer_valid = phase_fit_valid_;
    debug.phase_delta = phase_last_delta_;
    debug.phase_w_instant = phase_w_instant_;
    debug.phase_w_filtered = phase_w_fit_;
    debug.phase_w_applied = phase_w_applied_;
    debug.armor_switched = last_result_.armor_switched;
    debug.joint_pair_requested = last_result_.pair_requested;
    debug.joint_pair_used = last_result_.pair_used;
    debug.joint_second_id = last_result_.second_matched_id;
    debug.joint_nis = last_result_.joint_nis;
    debug.joint_second_position_error_m =
        last_result_.second_position_error;
    debug.joint_second_angle_error_rad =
        last_result_.second_angle_error;
    debug.joint_status = last_result_.pair_status;
    debug.candidate_is_switch = last_result_.armor_switched;
    debug.topology_event = last_result_.armor_switched;
    debug.geometry_update_allowed = last_result_.updated;
    debug.geometry_preserved = false;
    debug.geometry_valid = hasState();
    debug.armor_parity = last_result_.matched_id >= 0
                             ? last_result_.matched_id % 2
                             : -1;

    // 观测在内部是 m/SP 角度；导出时保留 m 但转换为项目 yaw 约定。
    if (last_observation_) {
        debug.measurement << last_observation_->xyz[0],
                             last_observation_->xyz[1],
                             last_observation_->xyz[2],
                             toProjectYaw(last_observation_->angle);
        debug.measurement_yaw = debug.measurement[3];
    }

    if (last_result_.matched_id >= 0) {
        debug.pre_predicted << last_result_.predicted_xyza[0],
                               last_result_.predicted_xyza[1],
                               last_result_.predicted_xyza[2],
                               toProjectYaw(last_result_.predicted_xyza[3]);
        debug.predicted_yaw = debug.pre_predicted[3];
        if (last_observation_) {
            debug.pre_residual =
                last_observation_->xyz - last_result_.predicted_xyza.head<3>();
            debug.pre_position_error = debug.pre_residual.norm();
            debug.yaw_innovation = wrapAngle(
                debug.measurement_yaw - debug.predicted_yaw);
        }
    }

    const sp_ekf::Target* target = tracker_->target();
    // 从后验状态和协方差抽取几何参数，供可视化定位问题。
    if (hasState() && target) {
        const Eigen::VectorXd x = target->ekfX();
        const Eigen::MatrixXd& P = target->ekf().P;
        debug.r1_m = x[8];
        debug.r2_m = x[8] + x[9];
        debug.h_m = x[10];
        debug.p_r1_m2 = P(8, 8);
        debug.p_r2_m2 = P(8, 8) + P(9, 9) + 2.0 * P(8, 9);
        debug.p_h_m2 = P(10, 10);
        debug.p_x_m2 = P(0, 0);
        debug.p_vx_m2_s2 = P(1, 1);
        debug.p_y_m2 = P(2, 2);
        debug.p_vy_m2_s2 = P(3, 3);

        if (last_result_.matched_id >= 0) {
            const auto armors = target->armorXyzaList();
            const std::size_t id =
                static_cast<std::size_t>(last_result_.matched_id);
            if (id < armors.size()) {
                const Eigen::Vector4d& post = armors[id];
                debug.post_predicted << post[0], post[1], post[2],
                                        toProjectYaw(post[3]);
                if (last_observation_) {
                    debug.post_residual = last_observation_->xyz - post.head<3>();
                    debug.post_position_error = debug.post_residual.norm();
                }
            }
        }
    }

    return debug;
}

bool SuperPowerPredictor::ready() const {
    return tracker_ && tracker_->ready();
}

bool SuperPowerPredictor::hasState() const {
    return tracker_ && tracker_->hasState();
}

void SuperPowerPredictor::warnTimeIssue(const char* reason,
                                       double update_time,
                                       double dt) {
    // 同一段异常时间序列只告警一次，避免日志被重复帧淹没。
    if (!timestamp_warning_active_) {
        std::cerr << "[SuperPowerPredictor] warning: " << reason
                  << "; t=" << update_time << " dt=" << dt << " s"
                  << std::endl;
        timestamp_warning_active_ = true;
    }
}

// 清除旧目标或异常时间段留下的相位样本，下一帧观测将重新建立解包基准。
void SuperPowerPredictor::resetAngularVelocityFit() {
    phase_samples_.clear();
    phase_reference_valid_ = false;
    phase_fit_valid_ = false;
    last_phase_wrapped_ = 0.0;
    unwrapped_phase_ = 0.0;
    phase_last_delta_ = 0.0;
    phase_w_instant_ = 0.0;
    phase_w_fit_ = 0.0;
    phase_w_applied_ = false;
}

// 先用 matched_id 扣除各装甲板的固定相位差，再对连续相位执行普通最小二乘拟合。
// 拟合斜率即角速度 w；样本不足或时间方差过小时保留当前角速度，不做回写。
void SuperPowerPredictor::observeAngularVelocity(double observation_time) {
    phase_w_applied_ = false;
    if (!last_observation_ || !tracker_ || !tracker_->hasState() ||
        last_result_.matched_id < 0 || config_.armor_num <= 0) {
        return;
    }

    const double armor_phase =
        last_result_.matched_id * 2.0 * kPi /
        static_cast<double>(config_.armor_num);
    const double phase_wrapped =
        wrapAngle(last_observation_->angle - armor_phase);

    if (!phase_reference_valid_) {
        phase_reference_valid_ = true;
        last_phase_wrapped_ = phase_wrapped;
        unwrapped_phase_ = phase_wrapped;
    } else {
        phase_last_delta_ = wrapAngle(phase_wrapped - last_phase_wrapped_);
        const double sample_dt = observation_time - phase_samples_.back().t;
        if (sample_dt > 1e-9) {
            phase_w_instant_ = phase_last_delta_ / sample_dt;
        }
        unwrapped_phase_ += phase_last_delta_;
        last_phase_wrapped_ = phase_wrapped;
    }

    phase_samples_.push_back({observation_time, unwrapped_phase_});
    while (phase_samples_.size() > 1 &&
           observation_time - phase_samples_.front().t >
               angular_velocity_fit_window_s_) {
        phase_samples_.pop_front();
    }

    if (phase_samples_.size() < angular_velocity_fit_min_samples_) return;

    double mean_t = 0.0;
    double mean_phase = 0.0;
    for (const PhaseSample& sample : phase_samples_) {
        mean_t += sample.t;
        mean_phase += sample.phase;
    }
    mean_t /= static_cast<double>(phase_samples_.size());
    mean_phase /= static_cast<double>(phase_samples_.size());

    double numerator = 0.0;
    double denominator = 0.0;
    for (const PhaseSample& sample : phase_samples_) {
        const double centered_t = sample.t - mean_t;
        numerator += centered_t * (sample.phase - mean_phase);
        denominator += centered_t * centered_t;
    }
    if (denominator <= 1e-12) return;

    phase_w_fit_ = numerator / denominator;
    phase_fit_valid_ = std::isfinite(phase_w_fit_);
    if (phase_fit_valid_) {
        tracker_->setAngularVelocity(phase_w_fit_);
        phase_w_applied_ = true;
    }
}

void SuperPowerPredictor::resetTracker() {
    // 重新创建 Tracker 同时清空其结果快照；时间状态由调用方单独复位。
    tracker_ = std::make_unique<sp_ekf::Tracker>(config_);
    last_result_ = sp_ekf::TrackerResult{};
    resetAngularVelocityFit();
}

void SuperPowerPredictor::initializeFromObservation(
    const EKFTargetObservation& observation) {
    if (!tracker_) resetTracker();
    // 以 dt=0 建立初始 Target，防止把首帧时间绝对值误当作时间间隔。
    last_observation_ = toSuperPower(observation);
    last_result_ = tracker_->process(last_observation_, 0.0);
    last_update_time_ = observation.t;
    last_dt_s_ = 0.0;
    has_update_time_ = true;
    update_frames_ = last_result_.initialized_this_frame ? 1 : 0;
    if (last_result_.matched_id >= 0) {
        debug_flip_flag_ = (last_result_.matched_id % 2) + 1;
    }
    if (last_result_.updated && last_result_.matched_id >= 0) {
        observeAngularVelocity(observation.t);
    }
}

sp_ekf::ArmorObservation SuperPowerPredictor::toSuperPower(
    const EKFTargetObservation& observation) {
    sp_ekf::ArmorObservation result;
    result.xyz << observation.x / kMillimetersPerMeter,
                  observation.y / kMillimetersPerMeter,
                  observation.z / kMillimetersPerMeter;

    // 项目几何：p = c + r*[sin(yaw), -cos(yaw)]。
    // SP 几何：p = c - r*[cos(angle), sin(angle)]。
    // 令 angle = yaw + pi/2 后，两种定义完全等价。
    result.angle = wrapAngle(observation.yaw + kHalfPi);
    return result;
}

double SuperPowerPredictor::toProjectYaw(double superpower_angle) {
    return wrapAngle(superpower_angle - kHalfPi);
}

double SuperPowerPredictor::wrapAngle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle <= -kPi) angle += 2.0 * kPi;
    return angle;
}
