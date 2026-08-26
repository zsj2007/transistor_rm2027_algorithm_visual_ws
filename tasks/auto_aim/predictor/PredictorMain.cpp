#include "predictor/PredictorMain.h"

void PredictorMain::update_serial_info(float bullet_velocity, float last_pitch_rad_delayed, float last_yaw_rad_delayed, float total_yaw_rad_delayed) {
    last_pitch_rad_delayed_ = last_pitch_rad_delayed;
    last_yaw_rad_delayed_ = last_yaw_rad_delayed;
    total_yaw_rad_delayed_ = total_yaw_rad_delayed;
    for (std::shared_ptr<AllPredictor>& all_predictor : all_predictors_) {
        if (all_predictor) {
            all_predictor -> update_serial_info(bullet_velocity, last_pitch_rad_delayed, last_yaw_rad_delayed, total_yaw_rad_delayed);
        }
    }
}

PredictorResult PredictorMain::step(
    std::vector<ArmorResult>& classifyResults,
    const std::vector<JointEkfTrackPair>& joint_pairs,
    cv::Mat& frame,
    double frame_timestamp_s,
                                    ArmorType::ArmorType priority_armor,
                                    bool auto_aim_switch,
                                    bool mcu_yaw_online) {

    PredictorResult chosen_result;

    TargetManagerUpdate target_update = target_manager_->update(
        classifyResults, frame.size(), frame_timestamp_s, priority_armor);

    if (target_update.released_target.has_value()) {
        const auto index =
            static_cast<std::size_t>(*target_update.released_target);
        if (index < all_predictors_.size() && all_predictors_[index]) {
            all_predictors_[index]->resetTarget();
        }
    }

    if (target_update.start_target.has_value()) {
        const auto index = static_cast<std::size_t>(*target_update.start_target);
        if (index < all_predictors_.size() && all_predictors_[index]) {
            all_predictors_[index]->resetTarget();
            all_predictors_[index]->startTarget();
        }
    }

    const TargetManagerStatus& target_status = target_manager_->status();
    if (target_update.process_current_frame &&
        target_status.target_type.has_value()) {
        const auto active_index =
            static_cast<std::size_t>(*target_status.target_type);
        if (active_index < all_predictors_.size() &&
            all_predictors_[active_index]) {
            // 每帧只更新当前锁定目标，其余识别结果仅用于可视化。
            if (!all_predictors_[active_index]->targetActive()) {
                all_predictors_[active_index]->resetTarget();
                all_predictors_[active_index]->startTarget();
            }
            chosen_result = all_predictors_[active_index]->step(
                target_update.target_candidates,
                joint_pairs,
                frame,
                frame_timestamp_s);
        }
    }

    // 目标管理器统一控制火控输出，只有连续跟踪状态允许开火和积分。
    const bool allow_tracking_output =
        target_update.process_current_frame &&
        TargetManager::allowsFireControl(target_status.state);
    if (!allow_tracking_output) {
        chosen_result.fire_flag = false;
        chosen_result.integrating = false;
    }

    if (auto_aim_switch) { // 仅在电控自瞄开关打开时进行积分
        if (chosen_result.integrating) {
            pitch_integration += std::min(std::max(chosen_result.command_delta_pitch,
                                    -command_picth_integration_max_speed_degree),
                                    command_picth_integration_max_speed_degree) * command_picth_integration_speed;
            yaw_integration += std::min(std::max(chosen_result.command_delta_yaw,
                                    -command_yaw_integration_max_speed_degree),
                                    command_yaw_integration_max_speed_degree) * command_yaw_integration_speed;
        }
        if (pitch_integration > pitch_integration_max_degree * M_PI / 180.0) {
            pitch_integration = pitch_integration_max_degree * M_PI / 180.0;
        }
        if (pitch_integration < pitch_integration_min_degree * M_PI / 180.0) {
            pitch_integration = pitch_integration_min_degree * M_PI / 180.0;
        }
        if (mcu_yaw_online) { // 电控yaw轴掉线时，关闭yaw轴积分重置及积分限制
            if (yaw_integration > yaw_integration_max_degree * M_PI / 180.0) {
                yaw_integration = yaw_integration_max_degree * M_PI / 180.0;
            }
            if (yaw_integration < yaw_integration_min_degree * M_PI / 180.0) {
                yaw_integration = yaw_integration_min_degree * M_PI / 180.0;
            }
        }
    } else {
        pitch_integration = 0.0;
        if (mcu_yaw_online) { // 电控yaw轴掉线时，关闭yaw轴积分重置及积分限制
            yaw_integration = 0.0;
        }
    }
    chosen_result.command_pitch = last_pitch_rad_delayed_ + chosen_result.command_delta_pitch * command_picth_kp + pitch_integration; // PI控制
    chosen_result.command_yaw = last_yaw_rad_delayed_ + chosen_result.command_delta_yaw * command_yaw_kp + yaw_integration; // 缓解yaw轴输入数据掉线问题（并不能()）


    cv::putText(frame, 
        mcu_yaw_online ? "mcu_yaw: online" : "mcu_yaw: offline", 
        cv::Point2f(20,870), 
        cv::FONT_HERSHEY_COMPLEX, 0.7, 
        mcu_yaw_online ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 1, 8, false);
    cv::putText(frame, 
        auto_aim_switch ? "auto_aim_switch_from_mcu: on" : "auto_aim_switch_from_mcu: off", 
        cv::Point2f(20,900), 
        cv::FONT_HERSHEY_COMPLEX, 0.7, 
        auto_aim_switch ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 1, 8, false);
    cv::putText(frame, 
        "pitch_integration: "+std::to_string(pitch_integration * 180.0 / M_PI), 
        cv::Point2f(20,930), 
        cv::FONT_HERSHEY_COMPLEX, 0.7, 
        cv::Scalar(0, 255, 0), 1, 8, false);
    cv::putText(frame, 
        "yaw_integration: "+std::to_string(yaw_integration * 180.0 / M_PI), 
        cv::Point2f(20,960), 
        cv::FONT_HERSHEY_COMPLEX, 0.7, 
        cv::Scalar(0, 255, 0), 1, 8, false);


    std::optional<std::string> ekf_state;
    if (target_status.target_type.has_value()) {
        const auto active_index =
            static_cast<std::size_t>(*target_status.target_type);
        if (active_index < all_predictors_.size() &&
            all_predictors_[active_index]) {
            ekf_state = all_predictors_[active_index]->ekfTrackerState();
        }
    }
    target_manager_->drawOverlay(frame, ekf_state);
    return chosen_result;
}

void PredictorMain::reset_yaw_integration() {
    yaw_integration = 0.0;
}

const TargetManagerStatus& PredictorMain::targetManagerStatus() const
{
    return target_manager_->status();
}
