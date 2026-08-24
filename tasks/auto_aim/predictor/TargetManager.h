#pragma once

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "2d_armor_detector/Armor.h"

enum class TargetState {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
    SWITCHING
};

struct TargetManagerStatus {
    TargetState state = TargetState::LOST;
    std::optional<ArmorType::ArmorType> target_type;
    int detect_count = 0;
    int temp_lost_count = 0;
    ArmorType::ArmorType acquire_policy = ArmorType::Nearest;
};

struct TargetManagerUpdate {
    std::vector<ArmorResult> target_candidates;
    std::optional<ArmorType::ArmorType> start_target;
    std::optional<ArmorType::ArmorType> released_target;
    bool process_current_frame = true;
};

class TargetManager {
public:
    explicit TargetManager(std::shared_ptr<YAML::Node> config_file_ptr);

    TargetManagerUpdate update(const std::vector<ArmorResult>& measurements,
                               const cv::Size& frame_size,
                               double frame_timestamp_s,
                               ArmorType::ArmorType acquire_policy);

    const TargetManagerStatus& status() const;
    void drawOverlay(cv::Mat& frame,
                     const std::optional<std::string>& ekf_state) const;

    static const char* stateName(TargetState state);
    static cv::Scalar stateColor(TargetState state);
    static bool allowsFireControl(TargetState state);

private:
    const ArmorResult* selectAcquisition(
        const std::vector<ArmorResult>& measurements,
        const cv::Size& frame_size,
        ArmorType::ArmorType acquire_policy) const;
    std::vector<ArmorResult> candidatesForTarget(
        const std::vector<ArmorResult>& measurements) const;
    int currentMaxTempLostFrames() const;
    void releaseTarget(TargetManagerUpdate& update);
    void clearTargetState();

    TargetManagerStatus status_;
    std::optional<double> last_frame_timestamp_s_;
    int min_detect_frames_ = 3;
    int max_temp_lost_frames_ = 15;
    int outpost_max_temp_lost_frames_ = 30;
    double max_frame_gap_s_ = 1.0;
    bool enable_priority_switch_ = false;
};
