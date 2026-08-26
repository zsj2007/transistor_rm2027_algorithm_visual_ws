#include "EKF/SuperPowerTracker.h"

#include <numeric>

namespace sp_ekf {

Tracker::Tracker(const TrackerConfig& config) : config_(config) {}

TrackerResult Tracker::process(
    const std::optional<ArmorObservation>& observation,
    double dt) {
    return processImpl(observation, std::nullopt, dt);
}

TrackerResult Tracker::processPair(
    const ArmorObservation& primary,
    const ArmorObservation& secondary,
    double dt) {
    return processImpl(primary, secondary, dt);
}

TrackerResult Tracker::processImpl(
    const std::optional<ArmorObservation>& primary,
    const std::optional<ArmorObservation>& secondary,
    double dt) {
    TrackerResult result;
    result.state_before = state_;
    result.measurement_valid = primary.has_value();

    if (state_ != TrackerState::LOST && dt > config_.max_dt_s) {
        state_ = TrackerState::LOST;
    }

    bool found = false;
    TargetUpdateDebug update_debug;

    if (state_ == TrackerState::LOST) {
        if (primary) {
            found = setTarget(*primary);
            result.initialized_this_frame = found;
            result.updated = found;
            if (found) {
                result.matched_id = 0;
                result.predicted_xyza = target_->armorXyzaList().front();
                if (secondary && config_.pair_update.enabled) {
                    result.pair_requested = true;
                    result.pair_status = "INITIALIZING";
                }
            }
        }
    } else if (primary) {
        // 同一帧无论单板还是双板都只做一次时间预测。
        target_->predict(dt);
        if (secondary && config_.pair_update.enabled) {
            update_debug =
                target_->updatePair(*primary, *secondary, config_.pair_update);
        } else {
            update_debug = target_->update(*primary);
        }
        found = true;
        result.updated = true;
        result.matched_id = update_debug.matched_id;
        result.armor_switched = update_debug.armor_switched;
        result.nis = update_debug.nis;
        result.position_error = update_debug.position_error;
        result.angle_error = update_debug.angle_error;
        result.predicted_xyza = update_debug.predicted_xyza;
        result.pair_requested = update_debug.pair_requested;
        result.pair_used = update_debug.pair_used;
        result.second_matched_id = update_debug.second_matched_id;
        result.joint_nis = update_debug.joint_nis;
        result.second_position_error =
            update_debug.second_position_error;
        result.second_angle_error = update_debug.second_angle_error;
        result.pair_status = update_debug.pair_status;
    } else {
        predictOnly(dt);
    }

    stateMachine(found);

    if (state_ != TrackerState::LOST && target_) {
        if (target_->diverged() || badConvergence()) {
            state_ = TrackerState::LOST;
        }
    }

    result.state = state_;
    result.lost_frames = temp_lost_count_;
    if (state_ == TrackerState::LOST && !target_) {
        result.matched_id = -1;
    }
    return result;
}

void Tracker::clear() {
    // 同时清空对象和所有帧计数，使下次观测走完整初始化流程。
    state_ = TrackerState::LOST;
    detect_count_ = 0;
    temp_lost_count_ = 0;
    target_.reset();
}

bool Tracker::setTarget(const ArmorObservation& observation) {
    // SP Tracker::set_target() 的普通四装甲初始化分支。
    target_.emplace(observation,
                    config_.initial_radius_m,
                    config_.armor_num,
                    normalFourArmorP0());
    return true;
}

void Tracker::predictOnly(double dt) {
    if (target_) target_->predict(dt);
}

void Tracker::stateMachine(bool found) {
    // 状态转移只依赖本帧是否有观测；滤波健康度在 process() 的后段单独检查。
    if (state_ == TrackerState::LOST) {
        if (!found) return;
        state_ = TrackerState::DETECTING;
        detect_count_ = 1;
        return;
    }

    if (state_ == TrackerState::DETECTING) {
        if (found) {
            ++detect_count_;
            if (detect_count_ >= config_.min_detect_count) {
                state_ = TrackerState::TRACKING;
            }
        } else {
            detect_count_ = 0;
            state_ = TrackerState::LOST;
        }
        return;
    }

    if (state_ == TrackerState::TRACKING) {
        if (found) return;
        temp_lost_count_ = 1;
        state_ = TrackerState::TEMP_LOST;
        return;
    }

    if (state_ == TrackerState::TEMP_LOST) {
        if (found) {
            state_ = TrackerState::TRACKING;
            temp_lost_count_ = 0;
        } else {
            ++temp_lost_count_;
            if (temp_lost_count_ > config_.max_temp_lost_count) {
                state_ = TrackerState::LOST;
            }
        }
    }
}

bool Tracker::badConvergence() const {
    if (!target_) return false;
    const auto& failures = target_->ekf().recent_nis_failures;
    const int sum = std::accumulate(failures.begin(), failures.end(), 0);
    // 最近窗口内至少 40% 的 NIS 超阈值，说明长期失配，应交还 LOST 重建。
    return sum >= static_cast<int>(0.4 * target_->ekf().window_size);
}

Eigen::VectorXd Tracker::normalFourArmorP0() {
    // 普通四装甲 11 维初始协方差对角线，数值与 SP 基线保持一致。
    Eigen::VectorXd diag(11);
    diag << 1.0, 64.0,
            1.0, 64.0,
            1.0, 64.0,
            0.4, 100.0,
            1.0, 1.0, 1.0;
    return diag;
}

const char* trackerStateName(TrackerState state) {
    switch (state) {
        case TrackerState::LOST: return "LOST";
        case TrackerState::DETECTING: return "DETECTING";
        case TrackerState::TRACKING: return "TRACKING";
        case TrackerState::TEMP_LOST: return "TEMP_LOST";
    }
    return "LOST";
}

}  // namespace sp_ekf
