#include "predictor/TargetManager.h"
#include "EKF/SuperPowerPredictor.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
bool isConcreteTargetType(int number)
{
    return number >= static_cast<int>(ArmorType::Hero) &&
           number <= static_cast<int>(ArmorType::Base);
}
std::string armorTypeName(ArmorType::ArmorType type)
{
    const auto index = static_cast<std::size_t>(type);
    if (index < ArmorType::ArmorTypeStrings.size()) return ArmorType::ArmorTypeStrings[index];
    return "UNKNOWN";
}
std::string acquisitionPolicyName(ArmorType::ArmorType type)
{
    if (type == ArmorType::Nearest) return "NEAREST";
    if (type == ArmorType::Middle) return "MIDDLE";
    return armorTypeName(type);
}
}  // namespace

TargetManager::TargetManager(std::shared_ptr<YAML::Node> config_file_ptr)
{
    const YAML::Node config = (*config_file_ptr)["target_manager"];
    min_detect_frames_ = std::max(1, config["min_detect_frames"].as<int>());
    max_temp_lost_frames_ = std::max(0, config["max_temp_lost_frames"].as<int>());
    outpost_max_temp_lost_frames_ = std::max(0, config["outpost_max_temp_lost_frames"].as<int>());
    enable_priority_switch_ = config["enable_priority_switch"].as<bool>();
    const YAML::Node reset_time_node = (*config_file_ptr)["reset_predictor_time"];
    if (reset_time_node) {
        const double reset_time_ms = reset_time_node.as<double>();
        if (std::isfinite(reset_time_ms) && reset_time_ms > 0.0) max_frame_gap_s_ = reset_time_ms / 1000.0;
    }
}

TargetManagerUpdate TargetManager::update(
    const std::vector<ArmorResult>& measurements,
    const cv::Size& frame_size,
    double frame_timestamp_s,
    ArmorType::ArmorType acquire_policy)
{
    // Publish the detector frame once, before the ordinary predictor runs.
    // SuperPower ignores this cache; Alliance consumes raw_light_bar_endpoints.
    SuperPowerPredictor::publishFrameMeasurements(measurements);

    TargetManagerUpdate update;
    if (!std::isfinite(frame_timestamp_s)) {
        update.process_current_frame = false;
        return update;
    }
    if (last_frame_timestamp_s_.has_value()) {
        const double dt = frame_timestamp_s - *last_frame_timestamp_s_;
        if (!std::isfinite(dt) || dt <= 0.0) {
            update.process_current_frame = false;
            return update;
        }
        if (dt > max_frame_gap_s_) {
            last_frame_timestamp_s_ = frame_timestamp_s;
            update.process_current_frame = false;
            if (status_.state != TargetState::LOST) releaseTarget(update);
            return update;
        }
    }
    last_frame_timestamp_s_ = frame_timestamp_s;
    status_.acquire_policy = acquire_policy;
    (void)enable_priority_switch_;

    if (status_.state == TargetState::LOST) {
        const ArmorResult* acquisition = selectAcquisition(measurements, frame_size, acquire_policy);
        if (acquisition == nullptr) return update;
        status_.target_type = static_cast<ArmorType::ArmorType>(acquisition->number);
        status_.state = TargetState::DETECTING;
        status_.detect_count = 1;
        status_.temp_lost_count = 0;
        update.start_target = status_.target_type;
        update.target_candidates = candidatesForTarget(measurements);
        return update;
    }

    if (!status_.target_type.has_value()) {
        clearTargetState();
        return update;
    }

    update.target_candidates = candidatesForTarget(measurements);
    const bool found = !update.target_candidates.empty();
    switch (status_.state) {
        case TargetState::DETECTING:
            if (!found) releaseTarget(update);
            else {
                ++status_.detect_count;
                if (status_.detect_count >= min_detect_frames_) status_.state = TargetState::TRACKING;
            }
            break;
        case TargetState::TRACKING:
            if (!found) { status_.state = TargetState::TEMP_LOST; status_.temp_lost_count = 1; }
            break;
        case TargetState::TEMP_LOST:
            if (found) { status_.state = TargetState::TRACKING; status_.temp_lost_count = 0; }
            else {
                ++status_.temp_lost_count;
                if (status_.temp_lost_count > currentMaxTempLostFrames()) releaseTarget(update);
            }
            break;
        case TargetState::SWITCHING:
            releaseTarget(update);
            break;
        case TargetState::LOST:
            break;
    }
    return update;
}

const TargetManagerStatus& TargetManager::status() const { return status_; }

void TargetManager::drawOverlay(cv::Mat& frame, const std::optional<std::string>& ekf_state) const
{
    if (frame.empty()) return;
    const cv::Scalar color = stateColor(status_.state);
    const int panel_width = std::min(470, std::max(0, frame.cols - 20));
    const int panel_height = ekf_state.has_value() ? 185 : 155;
    cv::rectangle(frame, cv::Rect(10,10,panel_width,panel_height), cv::Scalar(20,20,20), cv::FILLED);
    const std::string target_name = status_.target_type.has_value() ? armorTypeName(*status_.target_type) : "NONE";
    const std::string policy_name = acquisitionPolicyName(status_.acquire_policy);
    cv::putText(frame, "TARGET STATE: " + std::string(stateName(status_.state)), cv::Point(20,35), cv::FONT_HERSHEY_SIMPLEX, .68, color, 2, cv::LINE_AA);
    cv::putText(frame, "TARGET: " + target_name, cv::Point(20,65), cv::FONT_HERSHEY_SIMPLEX, .68, color, 2, cv::LINE_AA);
    cv::putText(frame, "detect: " + std::to_string(status_.detect_count), cv::Point(20,95), cv::FONT_HERSHEY_SIMPLEX, .62, color, 2, cv::LINE_AA);
    cv::putText(frame, "temp_lost: " + std::to_string(status_.temp_lost_count), cv::Point(20,125), cv::FONT_HERSHEY_SIMPLEX, .62, color, 2, cv::LINE_AA);
    cv::putText(frame, "acquire_policy: " + policy_name, cv::Point(20,155), cv::FONT_HERSHEY_SIMPLEX, .62, color, 2, cv::LINE_AA);
    if (ekf_state.has_value()) cv::putText(frame, "EKF STATE: " + *ekf_state, cv::Point(20,185), cv::FONT_HERSHEY_SIMPLEX, .62, cv::Scalar(255,255,0), 2, cv::LINE_AA);
}

const char* TargetManager::stateName(TargetState state)
{
    switch (state) {
        case TargetState::LOST:return "LOST"; case TargetState::DETECTING:return "DETECTING";
        case TargetState::TRACKING:return "TRACKING"; case TargetState::TEMP_LOST:return "TEMP_LOST";
        case TargetState::SWITCHING:return "SWITCHING";
    }
    return "UNKNOWN";
}
cv::Scalar TargetManager::stateColor(TargetState state)
{
    switch (state) {
        case TargetState::LOST:return cv::Scalar(0,0,255); case TargetState::DETECTING:return cv::Scalar(0,255,255);
        case TargetState::TRACKING:return cv::Scalar(0,255,0); case TargetState::TEMP_LOST:return cv::Scalar(0,165,255);
        case TargetState::SWITCHING:return cv::Scalar(255,0,255);
    }
    return cv::Scalar(255,255,255);
}
bool TargetManager::allowsFireControl(TargetState state) { return state == TargetState::TRACKING; }

const ArmorResult* TargetManager::selectAcquisition(
    const std::vector<ArmorResult>& measurements,
    const cv::Size& frame_size,
    ArmorType::ArmorType acquire_policy) const
{
    const ArmorResult* selected = nullptr;
    double best_score = std::numeric_limits<double>::infinity();
    const cv::Point2f image_center(static_cast<float>(frame_size.width)/2.0F, static_cast<float>(frame_size.height)/2.0F);
    for (const ArmorResult& candidate : measurements) {
        if (!isConcreteTargetType(candidate.number) || !candidate.solve_armor_result.valid) continue;
        if (acquire_policy != ArmorType::Nearest && acquire_policy != ArmorType::Middle &&
            candidate.number != static_cast<int>(acquire_policy)) continue;
        double score = 0.0;
        if (acquire_policy == ArmorType::Middle) score = cv::norm(candidate.center - image_center);
        else if (acquire_policy == ArmorType::Nearest) score = candidate.solve_armor_result.distance;
        else score = candidate.is_tracked_now ? -1.0e6-static_cast<double>(candidate.confidence) : -static_cast<double>(candidate.confidence);
        if (std::isfinite(score) && score < best_score) { best_score=score; selected=&candidate; }
    }
    return selected;
}

std::vector<ArmorResult> TargetManager::candidatesForTarget(const std::vector<ArmorResult>& measurements) const
{
    std::vector<ArmorResult> candidates;
    if (!status_.target_type.has_value()) return candidates;
    const int target_number = static_cast<int>(*status_.target_type);
    for (const ArmorResult& measurement : measurements) {
        if (measurement.number == target_number && measurement.solve_armor_result.valid) candidates.push_back(measurement);
    }
    return candidates;
}

int TargetManager::currentMaxTempLostFrames() const
{
    if (status_.target_type == ArmorType::Outpost) return outpost_max_temp_lost_frames_;
    return max_temp_lost_frames_;
}
void TargetManager::releaseTarget(TargetManagerUpdate& update)
{
    if (status_.target_type.has_value()) update.released_target = status_.target_type;
    clearTargetState();
}
void TargetManager::clearTargetState()
{
    status_.state=TargetState::LOST; status_.target_type.reset(); status_.detect_count=0; status_.temp_lost_count=0;
}
