#pragma once

#include <Eigen/Dense>
#include <optional>
#include <string>
#include <vector>

#include "EKF/SuperPowerTarget.h"

namespace sp_ekf {

enum class TrackerState {
    // LOST 无可用目标；DETECTING 累计初始检测；TRACKING 稳定跟踪；
    // TEMP_LOST 在短暂失检期间仅预测，等待重获。
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
};

struct TrackerConfig {
    // SuperPower standard3.yaml 的普通四装甲默认参数。
    int min_detect_count = 5;
    int max_temp_lost_count = 15;
    double max_dt_s = 0.1;
    double initial_radius_m = 0.2;
    int armor_num = 4;
    PairUpdateConfig pair_update;
};

struct TrackerResult {
    // process() 单帧结果：同时保留调用前后的状态，便于外层记录状态转移。
    TrackerState state = TrackerState::LOST;
    TrackerState state_before = TrackerState::LOST;
    bool initialized_this_frame = false;
    bool measurement_valid = false;
    bool updated = false;
    int lost_frames = 0;
    int matched_id = -1;
    bool armor_switched = false;
    double nis = -1.0;
    double position_error = -1.0;
    double angle_error = -1.0;
    Eigen::Vector4d predicted_xyza = Eigen::Vector4d::Zero();
    bool pair_requested = false;
    bool pair_used = false;
    int second_matched_id = -1;
    double joint_nis = -1.0;
    double second_position_error = -1.0;
    double second_angle_error = -1.0;
    std::string pair_status = "SINGLE";
};

class Tracker {
public:
    explicit Tracker(const TrackerConfig& config = TrackerConfig{});

    // 处理一帧可选观测：有观测时预测并更新，无观测时只预测。
    TrackerResult process(const std::optional<ArmorObservation>& observation,
                          double dt);
    TrackerResult processPair(
        const ArmorObservation& primary,
        const ArmorObservation& secondary,
        double dt);
    void clear();

    // SP 进入 LOST 后可能仍在内部缓存 Target，但不会把它提供给下游。
    // 此处保持同一可观察语义：LOST 一律视为没有可用状态。
    bool hasState() const {
        return state_ != TrackerState::LOST && target_.has_value();
    }
    bool ready() const {
        return state_ == TrackerState::TRACKING && target_.has_value();
    }
    TrackerState state() const { return state_; }
    const Target* target() const {
        return target_ ? &(*target_) : nullptr;
    }

private:
    // 状态机计数器：detect_count_ 用于确认建目标，temp_lost_count_ 用于
    // 限制仅预测的连续帧数。
    TrackerConfig config_;
    int detect_count_ = 0;
    int temp_lost_count_ = 0;
    TrackerState state_ = TrackerState::LOST;
    std::optional<Target> target_;

    // 状态机辅助操作，以及普通四装甲初始协方差的构造。
    TrackerResult processImpl(
        const std::optional<ArmorObservation>& primary,
        const std::optional<ArmorObservation>& secondary,
        double dt);
    bool setTarget(const ArmorObservation& observation);
    void predictOnly(double dt);
    void stateMachine(bool found);
    bool badConvergence() const;

    static Eigen::VectorXd normalFourArmorP0();
};

const char* trackerStateName(TrackerState state);

}  // namespace sp_ekf
