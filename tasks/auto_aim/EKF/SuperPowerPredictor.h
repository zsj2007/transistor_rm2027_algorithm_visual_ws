#pragma once

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "EKF/SuperPowerTracker.h"

struct EKFTargetObservation {
    // 项目坐标系下的装甲观测：位置单位 mm，yaw 单位 rad，时间单位 s。
    double x;
    double y;
    double z;
    double yaw;
    double t;
};

struct EKFPredictedArmor {
    // 预测装甲在项目坐标系下的位置、半径和朝向；长度单位均为 mm。
    double x;
    double y;
    double z;
    double r;
    double yaw;
};

struct EKFTargetPrediction {
    // 供弹道/火控层使用的前向外推结果；center 与 alternate_z 分别对应
    // 四块装甲中两种高度平面，r1/r2 为两组相对中心的半径。
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    double alternate_z = 0.0;
    double r1 = 0.0;
    double r2 = 0.0;
    double h = 0.0;
    double yaw = 0.0;
    double w = 0.0;
    int rotation_direction = 1;
    std::vector<EKFPredictedArmor> armors;
};

struct EKFTargetState {
    // 当前滤波后验状态的项目坐标系导出值；速度单位 mm/s，角速度单位 rad/s。
    double center_x = 0.0;
    double center_y = 0.0;
    double center_z = 0.0;
    double center_vx = 0.0;
    double center_vy = 0.0;
    double center_vz = 0.0;
    double r1 = 0.0;
    double r2 = 0.0;
    double h = 0.0;
    double yaw = 0.0;
    double w = 0.0;
    unsigned long long update_frames = 0;
};

// 导出给运行时可视化和火控层的 SuperPower 跟踪器诊断状态。
struct EKFTargetDebugState {
    // 本帧时间间隔及状态机/关联结果。
    double dt_s = 0.0;
    bool time_discontinuity = false;
    std::string tracker_state = "LOST";
    std::string tracker_state_before = "LOST";
    int matched_id = -1;
    bool measurement_valid = false;
    bool updated = false;
    int lost_frames = 0;
    double nis = -1.0;
    double position_error_m = -1.0;
    double yaw_error_deg = -1.0;
    bool phase_observer_valid = false;
    double phase_delta = 0.0;
    double phase_w_instant = 0.0;
    double phase_w_filtered = 0.0;
    bool direction_reversal = false;
    bool armor_switched = false;
    bool joint_pair_requested = false;
    bool joint_pair_used = false;
    int joint_second_id = -1;
    double joint_nis = std::numeric_limits<double>::quiet_NaN();
    double joint_second_position_error_m =
        std::numeric_limits<double>::quiet_NaN();
    double joint_second_angle_error_rad =
        std::numeric_limits<double>::quiet_NaN();
    std::string joint_status = "SINGLE";
    bool recovered = false;
    bool phase_w_applied = false;
    bool pending_sign_conflict = false;
    bool temp_lost_recovery = false;
    bool candidate_is_switch = false;
    bool topology_event = false;
    int best_id = -1;
    double measurement_yaw = std::numeric_limits<double>::quiet_NaN();
    double predicted_yaw = std::numeric_limits<double>::quiet_NaN();
    double yaw_innovation = std::numeric_limits<double>::quiet_NaN();
    // 观测、更新前预测、更新后预测依次为 [x, y, z, yaw]；位置单位 m，yaw 为项目约定。
    Eigen::Matrix<double, 4, 1> measurement =
        Eigen::Matrix<double, 4, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 4, 1> pre_predicted = measurement;
    Eigen::Matrix<double, 4, 1> post_predicted = measurement;
    Eigen::Matrix<double, 3, 1> pre_residual =
        Eigen::Matrix<double, 3, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 3, 1> post_residual = pre_residual;
    double pre_position_error = std::numeric_limits<double>::quiet_NaN();
    double post_position_error = std::numeric_limits<double>::quiet_NaN();
    double residual_radial = std::numeric_limits<double>::quiet_NaN();
    double residual_tangential = std::numeric_limits<double>::quiet_NaN();
    double nis_xyz = std::numeric_limits<double>::quiet_NaN();
    double nis_yaw = std::numeric_limits<double>::quiet_NaN();
    double yaw_variance_scale = 1.0;
    double hypothetical_scaled_nis = std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix<double, 4, 1> hypothetical_scaled_nis_contribution =
        Eigen::Matrix<double, 4, 1>::Constant(
            std::numeric_limits<double>::quiet_NaN());

    // 当前几何参数及关键协方差对角项，统一使用 SuperPower 内部单位 m。
    double r1_m = std::numeric_limits<double>::quiet_NaN();
    double r2_m = std::numeric_limits<double>::quiet_NaN();
    double h_m = std::numeric_limits<double>::quiet_NaN();
    double p_r1_m2 = std::numeric_limits<double>::quiet_NaN();
    double p_r2_m2 = std::numeric_limits<double>::quiet_NaN();
    double p_h_m2 = std::numeric_limits<double>::quiet_NaN();
    double p_x_m2 = std::numeric_limits<double>::quiet_NaN();
    double p_vx_m2_s2 = std::numeric_limits<double>::quiet_NaN();
    double p_y_m2 = std::numeric_limits<double>::quiet_NaN();
    double p_vy_m2_s2 = std::numeric_limits<double>::quiet_NaN();
    int armor_parity = -1;
    bool geometry_valid = false;
    bool geometry_update_allowed = false;
    bool geometry_preserved = false;
    int current_armor_id = -1;
};

// SuperPower 2025 普通四装甲 Target/Tracker/EKF 链路的运行时适配器。
// 内部严格保留 SP 的米、秒、弧度及装甲法线角约定；仅在边界处转换项目的
// 毫米、时间戳和 yaw 约定，避免把业务层坐标规则混入估计器本体。
class SuperPowerPredictor {
public:
    SuperPowerPredictor(const EKFTargetObservation& initial_observation,
                       double initial_radius_mm,
                       std::shared_ptr<YAML::Node> config_file_ptr);

    // 输入一条有效装甲观测；由观测时间戳计算 dt 后驱动跟踪器。
    void update(const EKFTargetObservation& observation);
    // 输入同一时间戳的两块相邻装甲观测。若联合门控失败，底层自动
    // 联合更新失败时回退为主观测的单板更新。
    void updatePair(const EKFTargetObservation& primary,
                    const EKFTargetObservation& secondary);
    // 无观测帧仅做状态预测，并让状态机累计临时丢失帧数。
    void missUpdate(double update_time);
    // 彻底清空跟踪器、时间基准和调试状态。
    void clear();

    // 从当前后验状态向未来 predict_time 秒做匀速/匀角速外推。
    EKFTargetPrediction predict(double predict_time) const;
    EKFTargetState state() const;
    EKFTargetDebugState debugState() const;

    bool ready() const;
    bool hasState() const;
    int debugFlipFlag() const { return debug_flip_flag_; }

private:
    // 项目输入与 SP 内部观测之间的单位、朝向约定转换。
    static sp_ekf::ArmorObservation toSuperPower(
        const EKFTargetObservation& observation);
    static double toProjectYaw(double superpower_angle);
    static double wrapAngle(double angle);

    struct PhaseSample {
        double t = 0.0;
        double phase = 0.0;
    };
    // 清空相位样本、解包基准和拟合诊断状态。
    void resetAngularVelocityFit();
    // 根据本帧 matched_id 还原车体相位，解包后用滑窗最小二乘拟合 w。
    // observation_time 为当前观测时间戳，单位为秒。
    void observeAngularVelocity(double observation_time);

    // 重建底层状态机；初始化函数用首条观测建立时间基准和初始 Target。
    void resetTracker();
    void initializeFromObservation(const EKFTargetObservation& observation);
    void updateImpl(
        const EKFTargetObservation& primary,
        const std::optional<EKFTargetObservation>& secondary);
    void warnTimeIssue(const char* reason, double update_time, double dt);

    // 从 YAML 读取的 SP 跟踪器配置，以及其持有的滤波状态机。
    sp_ekf::TrackerConfig config_;
    std::unique_ptr<sp_ekf::Tracker> tracker_;
    sp_ekf::TrackerResult last_result_;
    std::optional<sp_ekf::ArmorObservation> last_observation_;

    // 利用匹配到的物理装甲 ID 恢复连续车体相位，并在滑窗内拟合角速度 w。
    double angular_velocity_fit_window_s_ = 0.20;
    std::size_t angular_velocity_fit_min_samples_ = 4;
    std::deque<PhaseSample> phase_samples_;
    bool phase_reference_valid_ = false;
    bool phase_fit_valid_ = false;
    double last_phase_wrapped_ = 0.0;
    double unwrapped_phase_ = 0.0;
    double phase_last_delta_ = 0.0;
    double phase_w_instant_ = 0.0;
    double phase_w_fit_ = 0.0;
    bool phase_w_applied_ = false;

    // 时间戳保护：拒绝非有限、重复或乱序时间，避免异常 dt 污染 EKF。
    double last_update_time_ = 0.0;
    double last_dt_s_ = 0.0;
    unsigned long long update_frames_ = 0;
    int debug_flip_flag_ = 1;
    bool has_update_time_ = false;
    bool timestamp_warning_active_ = false;
    bool time_discontinuity_ = false;
};
