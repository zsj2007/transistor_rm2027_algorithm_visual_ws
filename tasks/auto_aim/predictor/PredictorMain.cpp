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

PredictorResult PredictorMain::step(std::vector<ArmorResult>& classifyResults, cv::Mat& frame, PredictorType::PredictorType predictor_type, ArmorType::ArmorType priority_armor, bool auto_aim_switch, bool mcu_yaw_online) {

    PredictorResult chosen_result;

    std::vector<std::vector<ArmorResult>> classified_classifyResults(classify_classes);
    for (ArmorResult& classify_result : classifyResults) {
        classified_classifyResults[classify_result.number].push_back(classify_result);
    }
    // todo 前哨站/基地特殊处理
    std::vector<PredictorResult> classified_predictor_results;
    for (size_t all_predictors_index = 0; all_predictors_index < classify_classes; all_predictors_index++) {
        if (classified_classifyResults[all_predictors_index].size() != 0) {
            if (all_predictors_[all_predictors_index] -> is_reset == true) {
                all_predictors_[all_predictors_index] -> latest_predicting_start_time = std::chrono::steady_clock::now();
                all_predictors_[all_predictors_index] -> is_reset = false;
            }
        }
        if (all_predictors_[all_predictors_index] -> is_reset == false) {
            classified_predictor_results.push_back(
                all_predictors_[all_predictors_index] -> step(classified_classifyResults[all_predictors_index], frame, predictor_type)
            );
            // RCLCPP_INFO(node->get_logger(), "%ld updating", all_predictors_index);
        }
    }

    if (priority_armor == ArmorType::Middle) {
        if (!classified_predictor_results.empty()) {
            auto it = std::min_element(
                classified_predictor_results.begin(), classified_predictor_results.end(),
                [](const PredictorResult& a, const PredictorResult& b) {
                    return a.pixel_horizontal_center_distance < b.pixel_horizontal_center_distance;
                }
            );
            if (it != classified_predictor_results.end()) {
                auto middle_result = *it;
                chosen_result = middle_result;
            }
        }
    } else if (priority_armor == ArmorType::Nearest) {
        if (!classified_predictor_results.empty()) {
            auto it = std::min_element(
                classified_predictor_results.begin(), classified_predictor_results.end(),
                [](const PredictorResult& a, const PredictorResult& b) {
                    return a.latest_armor_distance < b.latest_armor_distance;
                }
            );
            if (it != classified_predictor_results.end()) {
                auto nearest_result = *it;
                chosen_result = nearest_result;
            }
        }
    } else {
        for (PredictorResult predictor_result : classified_predictor_results) {
            if (predictor_result.armor_type == priority_armor && !predictor_result.reset) {
                chosen_result = predictor_result;
            }
        }
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


    return chosen_result;
}

void PredictorMain::reset_yaw_integration() {
    yaw_integration = 0.0;
}