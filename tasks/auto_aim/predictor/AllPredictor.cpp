#include "predictor/AllPredictor.h"

#include "tools/logger.hpp"

void AllPredictor::update_serial_info(float bullet_velocity, float last_pitch_rad_delayed, float last_yaw_rad_delayed, float total_yaw_rad_delayed) {
    bullet_velocity_ = bullet_velocity;
    last_pitch_rad_delayed_ = last_pitch_rad_delayed;
    last_yaw_rad_delayed_ = last_yaw_rad_delayed;
    total_yaw_rad_delayed_ = total_yaw_rad_delayed;
}

PredictorResult AllPredictor::step(std::vector<ArmorResult>& classifyResults, cv::Mat& frame, PredictorType::PredictorType control_predictor_type)
{
    PredictorResult result;

    bool ballistic_valid_flag = false;
    float total_delay = last_total_delay_;
    cv::Point3f predicted_armor_pos;
    cv::Point3f predicted_aim_pos;
    bool fire_flag = false;
    std::vector<float> cam_position = rest_frame_ -> getCamPosition();
    result.integrating = true;


    if (armor_class != ArmorType::Base) {
        // using_predictor_type = predictor_switcher_ -> step();
        if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - latest_predicting_start_time).count()
            < pre_predict_time) {

            using_predictor_type = PredictorType::None;
        } else {
            using_predictor_type = predictor_switcher_ -> step();
        }
    }
    if (!classifyResults.empty()) {
        // 选择最佳目标（置信度最高）
        auto it = std::max_element(
            classifyResults.begin(), classifyResults.end(),
            [](const ArmorResult& a, const ArmorResult& b) {
                if (a.is_tracked_now && !b.is_tracked_now) {
                    return false;
                }
                if (!a.is_tracked_now && b.is_tracked_now) {
                    return true;
                }
                return a.confidence < b.confidence;
            }
        );
        if (it != classifyResults.end()) {
            // 只读引用，避免按值拷贝 ArmorResult（内含多个 vector/Mat）
            const ArmorResult& chosen_armor = *it;
            const AimResult& solve_armor_result = chosen_armor.solve_armor_result;
            armor_is_large = chosen_armor.is_large;

            is_reset = false;
            last_com_time = std::chrono::steady_clock::now();

            last_pixel_horizontal_center_distance = std::abs(chosen_armor.center.x - static_cast<float>(frame.cols)/2.0);
            latest_armor_distance = std::sqrt(solve_armor_result.distance);
            
            // 查看z轴距离轴数据
            oscilloscope_common_ -> addDataPoint(solve_armor_result.position.z / 10000, 0);

            // 将pnp结果转换至静止坐标系以稳定预测
            cv::Point3f rest_frame_pos = rest_frame_ -> pnpToWorldP3f(solve_armor_result.position);
            // std::vector<float> rest_frame_euler_angles = {
            //     static_cast<float>(solve_armor_result.ba_global_ypr[0]),
            //     static_cast<float>(solve_armor_result.ba_global_ypr[1]),
            //     static_cast<float>(solve_armor_result.ba_global_ypr[2])
            // };
            std::vector<float> rest_frame_euler_angles = rest_frame_ -> getWorldEulerAnglesFromCam(
                solve_armor_result.normal_euler_angles[0], solve_armor_result.normal_euler_angles[1], solve_armor_result.normal_euler_angles[2]);

            tools::logger()->debug(
              "camera euler angles: yaw={:.2f}, pitch={:.2f}, roll={:.2f}",
              solve_armor_result.normal_euler_angles[0], solve_armor_result.normal_euler_angles[1],
              solve_armor_result.normal_euler_angles[2]);
            tools::logger()->debug(
              "Rest frame pos: x={:.2f}, y={:.2f}, z={:.2f}, yaw={:.2f}",
              rest_frame_pos.x, rest_frame_pos.y, rest_frame_pos.z, rest_frame_euler_angles[0]);

            last_rest_frame_pos = rest_frame_pos;

            // 提前预测与弹道解算
            float bullet_time = (bullet_velocity_ > 1.0f) ? (std::abs(solve_armor_result.position.z) / 1000.0f / bullet_velocity_) : 0.0f;
            total_delay = bullet_time + extra_predict_time;
            last_total_delay_ = total_delay;

            // 默认使用 None （直接瞄准装甲板）
            predicted_armor_pos = rest_frame_pos;
            predicted_aim_pos = predicted_armor_pos;
            fire_flag = true;

            if (!chosen_armor.is_tracked_now) {
                result.command_delta_pitch = 0.0;
                result.command_delta_yaw = 0.0;
                predicted_armor_pos = last_rest_frame_pos;
                predicted_aim_pos = last_rest_frame_pos;
            }
        }
    } else {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_com_time).count() >= reset_predictor_time) {
            if (rotation_motion_model_) {
                RotationMotionState RMMstate = rotation_motion_model_ -> getState();
                if (RMMstate.update_frames > 90) init_r = (RMMstate.r_now, RMMstate.r_another) / 2.0;
                if (!(init_r >= 200.0)) init_r = 200.0; // 限位，同时防止nan传播
                if (!(init_r <= 400.0)) init_r = 400.0;
                rotation_motion_model_.reset();
            }
            is_reset = true;
            last_pixel_horizontal_center_distance = 1e10;
            has_valid_ballistic = false;
            RMM_fire_control_data.new_target = true;
            latest_armor_distance = 1e10;

            result.reset = true;
            result.command_delta_pitch = 0.0;
            result.command_delta_yaw = 0.0;
            result.fire_flag = false;
            result.predictor_type = using_predictor_type;
            result.armor_type = armor_class;
            result.pixel_horizontal_center_distance = last_pixel_horizontal_center_distance;
            result.latest_armor_distance = latest_armor_distance;
            return result;
        } else {
            result.reset = false;
            result.command_delta_pitch = 0.0;
            result.command_delta_yaw = 0.0;
            result.fire_flag = false;
            predicted_armor_pos = last_rest_frame_pos;
            predicted_aim_pos = last_rest_frame_pos;
        }
    }





    if (armor_class != ArmorType::Base) {
        // ========================== RotationMotionModel ===========================
        double RMM_update_time = (std::chrono::steady_clock::now() - node_start_time).count() / 1e9;
        bool RMM_updated_flag = false;
        cv::Mat RMM_visualize_frame = cv::Mat::zeros(800, 800, CV_8UC3);
        bool has_tracked_armor_flag = false;
        if (!classifyResults.empty()) {
            for (auto& chosen_armor : classifyResults) {
                AimResult solve_armor_result = chosen_armor.solve_armor_result;
                cv::Point3f rest_frame_pos = rest_frame_ -> pnpToWorldP3f(solve_armor_result.position);
                // std::vector<float> rest_frame_euler_angles = {
                //     static_cast<float>(solve_armor_result.ba_global_ypr[0]),
                //     static_cast<float>(solve_armor_result.ba_global_ypr[1]),
                //     static_cast<float>(solve_armor_result.ba_global_ypr[2])
                // };
                std::vector<float> rest_frame_euler_angles = rest_frame_ -> getWorldEulerAnglesFromCam(
                    solve_armor_result.normal_euler_angles[0], solve_armor_result.normal_euler_angles[1], solve_armor_result.normal_euler_angles[2]);
                
                ObservedData RMM_update_data({
                    rest_frame_pos.x, rest_frame_pos.y, rest_frame_pos.z, rest_frame_euler_angles[0],
                    RMM_update_time
                });
                if (!rotation_motion_model_) {
                    rotation_motion_model_ = std::make_unique<RotationMotionModel>(RMM_update_data, rest_frame_, armor_class==ArmorType::Outpost, init_r);
                } else {
                    RotationMotionState RMM_state_to_check = rotation_motion_model_ -> getState();
                    if (
                        !(RMM_state_to_check.center_x > -1e10 && RMM_state_to_check.center_x < 1e10) ||
                        !(RMM_state_to_check.center_y > -1e10 && RMM_state_to_check.center_y < 1e10) ||
                        !(RMM_state_to_check.center_z > -1e10 && RMM_state_to_check.center_z < 1e10) ||
                        !(RMM_state_to_check.z_another > -1e10 && RMM_state_to_check.z_another < 1e10) ||
                        !(RMM_state_to_check.center_vx > -1e10 && RMM_state_to_check.center_vx < 1e10) ||
                        !(RMM_state_to_check.center_vy > -1e10 && RMM_state_to_check.center_vy < 1e10) ||
                        !(RMM_state_to_check.center_vz > -1e10 && RMM_state_to_check.center_vz < 1e10) ||
                        !(RMM_state_to_check.r_now > -1e10 && RMM_state_to_check.r_now < 1e10) ||
                        !(RMM_state_to_check.r_another > -1e10 && RMM_state_to_check.r_another < 1e10) ||
                        !(RMM_state_to_check.yaw > -1e10 && RMM_state_to_check.yaw < 1e10) ||
                        !(RMM_state_to_check.total_yaw > -1e10 && RMM_state_to_check.total_yaw < 1e10) ||
                        !(RMM_state_to_check.vyaw > -1e10 && RMM_state_to_check.vyaw < 1e10)
                    ) {
                        rotation_motion_model_.reset();
                        rotation_motion_model_ = std::make_unique<RotationMotionModel>(RMM_update_data, rest_frame_, armor_class==ArmorType::Outpost, init_r);
                    } // 出现nan立即重置

                    if (chosen_armor.is_tracked_now) {
                        rotation_motion_model_ -> update(RMM_update_data);
                        RMM_updated_flag = true;
                        has_tracked_armor_flag = true;
                    }
                }
                cv::circle(RMM_visualize_frame, cv::Point2f(400+rest_frame_pos.x/RMM_visualize_zoom_out_factor, 400-rest_frame_pos.y/RMM_visualize_zoom_out_factor), 8, cv::Scalar(255, 255, 0), 2);
                cv::line(RMM_visualize_frame, 
                    cv::Point2f(400 + rest_frame_pos.x/RMM_visualize_zoom_out_factor, 400-rest_frame_pos.y/RMM_visualize_zoom_out_factor), 
                    cv::Point2f(400 + rest_frame_pos.x/RMM_visualize_zoom_out_factor + std::sin(rest_frame_euler_angles[0])*500/RMM_visualize_zoom_out_factor, 
                                400 - (rest_frame_pos.y/RMM_visualize_zoom_out_factor - std::cos(rest_frame_euler_angles[0])*500/RMM_visualize_zoom_out_factor)),
                    cv::Scalar(255, 255, 0), 2);
                double theoretic_yaw = rotation_motion_model_ -> getTheoreticYaw(rest_frame_pos.x, rest_frame_pos.y);
                cv::line(RMM_visualize_frame, 
                    cv::Point2f(400 + rest_frame_pos.x/RMM_visualize_zoom_out_factor, 400-rest_frame_pos.y/RMM_visualize_zoom_out_factor), 
                    cv::Point2f(400 + rest_frame_pos.x/RMM_visualize_zoom_out_factor + std::sin(theoretic_yaw)*500/RMM_visualize_zoom_out_factor, 
                                400 - (rest_frame_pos.y/RMM_visualize_zoom_out_factor - std::cos(theoretic_yaw)*500/RMM_visualize_zoom_out_factor)),
                    cv::Scalar(0, 255, 0), 2);
            }
        } 
        if (!RMM_updated_flag) {
            if (rotation_motion_model_) {
                rotation_motion_model_ -> emptyUpdate(RMM_update_time);
            }
        }



        if(rotation_motion_model_) {
            PredictResult RMM_pred_now_data = rotation_motion_model_ -> predict(0.0);
            cv::Point3f RMM_pred_now_center_p3f = rest_frame_ -> worldToPnpP3f({
                static_cast<float>(RMM_pred_now_data.center_x), 
                static_cast<float>(RMM_pred_now_data.center_y), 
                static_cast<float>(RMM_pred_now_data.center_z)
            });
            cv::Point2f RMM_pred_now_center_pixel = armor_solver_->project3DToPixel(RMM_pred_now_center_p3f);
            if (has_tracked_armor_flag) {
                cv::circle(frame, RMM_pred_now_center_pixel, 10, cv::Scalar(0, 255, 0), 2);
            } else {
                cv::circle(frame, RMM_pred_now_center_pixel, 10, cv::Scalar(255, 0, 255), 2);
            }
            if (has_tracked_armor_flag) {
                cv::circle(RMM_visualize_frame, cv::Point2f(400+RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor), 8, cv::Scalar(0, 255, 0), 2);
            } else {
                cv::circle(RMM_visualize_frame, cv::Point2f(400+RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor), 8, cv::Scalar(255, 0, 255), 2);
            }
            for (int RMM_pred_now_armor_i = 0; RMM_pred_now_armor_i < RMM_pred_now_data.armors.size(); RMM_pred_now_armor_i += 1) {
                SimpleArmor& RMM_pred_now_armor = RMM_pred_now_data.armors[RMM_pred_now_armor_i];
                cv::Point3f RMM_pred_now_armor_p3f = rest_frame_ -> worldToPnpP3f({
                    static_cast<float>(RMM_pred_now_armor.x), 
                    static_cast<float>(RMM_pred_now_armor.y), 
                    static_cast<float>(RMM_pred_now_armor.z)
                });
                cv::Point2f RMM_pred_now_armor_pixel = armor_solver_->project3DToPixel(RMM_pred_now_armor_p3f);
                cv::circle(frame, RMM_pred_now_armor_pixel, 6, cv::Scalar(0, 255, 0), 2);
                // cv::line(frame, RMM_pred_now_center_pixel, RMM_pred_now_armor_pixel, cv::Scalar(0, 255, 0), 2);
                
                cv::circle(RMM_visualize_frame, cv::Point2f(400+RMM_pred_now_armor.x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_armor.y/RMM_visualize_zoom_out_factor), 8, 
                    cv::Scalar(255 - RMM_pred_now_armor_i * 60, 0, RMM_pred_now_armor_i * 60), 2);
                // cv::line(RMM_visualize_frame, 
                //     cv::Point2f(400+RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor), 
                //     cv::Point2f(400+RMM_pred_now_armor.x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_armor.y/RMM_visualize_zoom_out_factor), 
                //     cv::Scalar(255 - RMM_pred_now_armor_i * 60, 0, RMM_pred_now_armor_i * 60), 2);
            }

            RotationMotionState RMM_state = rotation_motion_model_ -> getState();
            cv::putText(RMM_visualize_frame, 
                "RMM_state vyaw:"+std::to_string(RMM_state.vyaw), 
                cv::Point2f(20,80), 
                cv::FONT_HERSHEY_COMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 1, 8, false);
            cv::putText(RMM_visualize_frame, 
                "T:"+std::to_string(RMM_update_time), 
                cv::Point2f(20,110), 
                cv::FONT_HERSHEY_COMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 1, 8, false);
            if (using_predictor_type == PredictorType::RotationMotionModel) {
                PredictResult RMM_pred_aim_data = rotation_motion_model_ -> predict(total_delay);
                cv::Point2d cam_to_center_vector = {RMM_pred_aim_data.center_x - cam_position[0], RMM_pred_aim_data.center_y - cam_position[1]};
                double cam_to_center_vector_norm = cv::norm(cam_to_center_vector);
                cv::Point2d unit_cam_to_center_vector = cv::Point2d(0.0, 1.0);
                if (cam_to_center_vector_norm > 1e-3) {
                    unit_cam_to_center_vector = cam_to_center_vector / cam_to_center_vector_norm;
                }
                std::vector<double> unit_center_v_dot_yaw(RMM_pred_aim_data.armors.size());
                float choose_armor_yaw_bias_with_direction = choose_armor_yaw_bias;
                choose_armor_yaw_bias_with_direction *= static_cast<float>(RMM_pred_aim_data.rotation_direction);
                for (int RMM_pred_aim_armor_i = 0; RMM_pred_aim_armor_i < RMM_pred_aim_data.armors.size(); RMM_pred_aim_armor_i += 1) {
                    SimpleArmor& RMM_pred_aim_armor = RMM_pred_aim_data.armors[RMM_pred_aim_armor_i];
                    cv::Point2d yaw_vector = {std::sin(RMM_pred_aim_armor.yaw + choose_armor_yaw_bias_with_direction), -std::cos(RMM_pred_aim_armor.yaw + choose_armor_yaw_bias_with_direction)};
                    unit_center_v_dot_yaw[RMM_pred_aim_armor_i] = cam_to_center_vector.dot(yaw_vector);
                }
                std::pair<int, int> nearest_two_idx = findTwoSmallestIndices(unit_center_v_dot_yaw);
                auto nearest_armor = RMM_pred_aim_data.armors[nearest_two_idx.first];
                auto second_nearest_armor = RMM_pred_aim_data.armors[nearest_two_idx.second];
                auto chosen_armor = nearest_armor;

                if (abs(RMM_state.vyaw) < RMM_fire_control_data.low_vyaw_threshold) {
                    cv::Point2d yaw_vector_1 = {std::sin(nearest_armor.yaw), -std::cos(nearest_armor.yaw)};
                    double yaw_bias_1 = acos(-unit_cam_to_center_vector.dot(yaw_vector_1));
                    cv::Point2d yaw_vector_2 = {std::sin(second_nearest_armor.yaw), -std::cos(second_nearest_armor.yaw)};
                    double yaw_bias_2 = acos(-unit_cam_to_center_vector.dot(yaw_vector_2));
                    if (abs(yaw_bias_1 - yaw_bias_2) < RMM_fire_control_data.low_vyaw_change_target_delta_yaw_threshold) {
                        float delta_yaw_1 = nearest_armor.yaw - RMM_fire_control_data.last_target_yaw;
                        delta_yaw_1 = atan2(sin(delta_yaw_1), cos(delta_yaw_1));
                        float delta_yaw_2 = second_nearest_armor.yaw - RMM_fire_control_data.last_target_yaw;
                        delta_yaw_2 = atan2(sin(delta_yaw_2), cos(delta_yaw_2));
                        if (abs(delta_yaw_1) < abs(delta_yaw_2)) {
                            chosen_armor = nearest_armor;
                        } else {
                            chosen_armor = second_nearest_armor;
                        }
                    }
                }

                predicted_armor_pos = {
                    static_cast<float>(chosen_armor.x),
                    static_cast<float>(chosen_armor.y),
                    static_cast<float>(chosen_armor.z) 
                };

                float chosen_armor_yaw_bias = (chosen_armor.yaw - (rotation_motion_model_ -> getCamToCenterYaw())) * static_cast<float>(RMM_pred_aim_data.rotation_direction);
                chosen_armor_yaw_bias = atan2(sin(chosen_armor_yaw_bias), cos(chosen_armor_yaw_bias));

                RMM_fire_result_t RMM_fire_result = RMM_fire_control(chosen_armor, RMM_state, chosen_armor_yaw_bias, armor_is_large, cam_to_center_vector, choose_armor_yaw_bias_with_direction);
                if (RMM_fire_result.aim_center) {
                    cv::Point2d reverse_straight_r_vector = unit_cam_to_center_vector * chosen_armor.r;
                    cv::Point2d cam_to_center_vector_subtract_r = cam_to_center_vector - reverse_straight_r_vector;
                    predicted_aim_pos = {
                        static_cast<float>(cam_position[0] + cam_to_center_vector_subtract_r.x),
                        static_cast<float>(cam_position[1] + cam_to_center_vector_subtract_r.y),
                        static_cast<float>(chosen_armor.z) 
                    };
                } else {
                    predicted_aim_pos = predicted_armor_pos;
                }
                fire_flag = RMM_fire_result.fire;
                
                cv::circle(RMM_visualize_frame, 
                    cv::Point2f(400+chosen_armor.x/RMM_visualize_zoom_out_factor, 400-chosen_armor.y/RMM_visualize_zoom_out_factor), 8, 
                    cv::Scalar(0, 0, 255), 2);
                cv::circle(RMM_visualize_frame, 
                    cv::Point2f(400+predicted_aim_pos.x/RMM_visualize_zoom_out_factor, 400-predicted_aim_pos.y/RMM_visualize_zoom_out_factor), 8, 
                    cv::Scalar(0, 255, 255), 2);
                cv::putText(RMM_visualize_frame, 
                    "r_now:"+std::to_string(RMM_pred_aim_data.r_now), 
                    cv::Point2f(20,140), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
                cv::putText(RMM_visualize_frame, 
                    "r_another:"+std::to_string(RMM_pred_aim_data.r_another), 
                    cv::Point2f(300,140), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
                cv::putText(RMM_visualize_frame, 
                    "flip:"+std::to_string(rotation_motion_model_ -> debug_flip_flag), 
                    cv::Point2f(580,140), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
                cv::putText(RMM_visualize_frame, 
                    "center_z:"+std::to_string(RMM_pred_aim_data.center_z), 
                    cv::Point2f(20,170), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
                cv::putText(RMM_visualize_frame, 
                    "z_another:"+std::to_string(RMM_pred_aim_data.z_another), 
                    cv::Point2f(300,170), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
                cv::putText(RMM_visualize_frame, 
                    "aim_center:"+std::to_string(RMM_fire_result.aim_center), 
                    cv::Point2f(20,290), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
            }
            cv::line(RMM_visualize_frame, 
                cv::Point2f(400, 400), 
                cv::Point2f(400 - std::sin(total_yaw_rad_delayed_)*1500/RMM_visualize_zoom_out_factor, 
                            400 - std::cos(total_yaw_rad_delayed_)*1500/RMM_visualize_zoom_out_factor),
                cv::Scalar(255, 255, 0), 2);
            cv::putText(RMM_visualize_frame, 
                "total_yaw:"+std::to_string(total_yaw_rad_delayed_), 
                cv::Point2f(20,200), 
                cv::FONT_HERSHEY_COMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 1, 8, false);
            cv::putText(RMM_visualize_frame, 
                "vx:"+std::to_string(RMM_state.center_vx), 
                cv::Point2f(20,230), 
                cv::FONT_HERSHEY_COMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 1, 8, false);
            cv::putText(RMM_visualize_frame, 
                "vy:"+std::to_string(RMM_state.center_vy), 
                cv::Point2f(20,260), 
                cv::FONT_HERSHEY_COMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 1, 8, false);
            cv::line(RMM_visualize_frame, 
                cv::Point2f(400 + RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor, 400 - RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor), 
                cv::Point2f(400 + (RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor + RMM_state.center_vx*2/RMM_visualize_zoom_out_factor), 
                            400 - (RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor + RMM_state.center_vy*2/RMM_visualize_zoom_out_factor)),
                cv::Scalar(255, 255, 0), 2);
            cv::line(RMM_visualize_frame, 
                cv::Point2f(400+RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor),
                cv::Point2f(400+RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor + std::sin(RMM_state.total_yaw)*1000/RMM_visualize_zoom_out_factor, 
                            400-RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor + std::cos(RMM_state.total_yaw)*1000/RMM_visualize_zoom_out_factor),
                cv::Scalar(255, 0, 255), 2);

            cv::putText(RMM_visualize_frame, 
                "Class:"+std::to_string(armor_class)+"("+ArmorType::ArmorTypeStrings[armor_class]+")", 
                cv::Point2f(20,50), 
                cv::FONT_HERSHEY_COMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 1, 8, false);
// #ifdef SHOW_WINDOWS
//             cv::imshow("RMM visualize "+std::to_string(armor_class), RMM_visualize_frame);
// #endif
            result.info_images.RMM_visualize_frame = RMM_visualize_frame;
        }
        // ========================== RotationMotionModsel =========================== END
    }

    // 统一转换回pnp相机坐标系    
    predicted_aim_pos = rest_frame_ -> worldToPnpP3f(predicted_aim_pos);
    predicted_armor_pos = rest_frame_ -> worldToPnpP3f(predicted_armor_pos);

    // 弹道解算
    tools::logger()->debug(
      "aim pos: ({:.2f}, {:.2f}, {:.2f})", predicted_aim_pos.x, predicted_aim_pos.y, predicted_aim_pos.z);
    BallisticInfo ballistic_result = ballistic_solver_ -> calcBallisticAngle(
        predicted_aim_pos.x, 
        predicted_aim_pos.y, 
        predicted_aim_pos.z,
        bullet_velocity_,
        last_pitch_rad_delayed_,//pitch_integration | last_pitch_rad_delayed_ #todo
        last_yaw_rad_delayed_
    );
    
    if (ballistic_result.valid) {
        ballistic_valid_flag = true;
        has_valid_ballistic = true;
        // RCLCPP_INFO(node->get_logger(), "Target detected, publishing command");
        // has_valid_target_ = true;
        
        // 发布云台控制命令
        //serial_communication_->sendData(command_pitch, command_yaw, fire_flag);
        result.reset = false;
        result.command_delta_pitch = ballistic_result.delta_pitch_rad;
        result.command_delta_yaw = ballistic_result.delta_yaw_rad;
        result.fire_flag = fire_flag;
        
        // 绘制瞄准预测点（黄色）
        cv::Point2f pred_aim_pixel = armor_solver_->project3DToPixel(predicted_aim_pos);
        cv::circle(frame, pred_aim_pixel, 8, cv::Scalar(0, 255, 255), 2);
        
        cv::Point2f pred_armor_pixel = armor_solver_->project3DToPixel(predicted_armor_pos);
        // 绘制装甲板预测点（天蓝色）
        cv::circle(frame, pred_armor_pixel, 8, cv::Scalar(255, 255, 0), 2);
    }





    if ((!ballistic_valid_flag) && has_valid_ballistic) {
        result.reset = false;
        result.command_delta_pitch = 0.0;
        result.command_delta_yaw = 0.0;
        result.fire_flag = false;
    }





    // 计算并绘制瞄准时目标画面中心（天蓝色：未开火 | 红色：开火）
    cv::Point2f aim_yaw_pitch = cv::Point2f(last_yaw_rad_delayed_ + result.command_delta_yaw, last_pitch_rad_delayed_ + result.command_delta_pitch);
    cv::Point2f aim_yaw_pitch_pixel = cv::Point2f(
        frame.cols / 2 - (aim_yaw_pitch.x - last_yaw_rad_delayed_) * yaw_rad_to_x_pixel_ratio, 
        frame.rows / 2 - (aim_yaw_pitch.y - last_pitch_rad_delayed_) * pitch_rad_to_y_pixel_ratio);
    last_aim_yaw_pitch_ = aim_yaw_pitch;
    if (result.fire_flag) {
        cv::circle(frame, aim_yaw_pitch_pixel, 8, cv::Scalar(0, 0, 255), 2);
    } else {
        cv::circle(frame, aim_yaw_pitch_pixel, 8, cv::Scalar(255, 255, 0), 2);
    }
    tools::logger()->debug("aim center yaw pitch: ({:.2f}, {:.2f})", aim_yaw_pitch.x, aim_yaw_pitch.y);
    
    oscilloscope_common_ -> addDataPoint(((float)(result.fire_flag))/11.0, 1);
    oscilloscope_common_ -> update();
    // oscilloscope_common_ -> show();
    result.info_images.common_debug_oscilloscope_frame = oscilloscope_common_ -> getDisplay();

    std::string using_predictor_type_string = PredictorType::PredictorTypeStrings[using_predictor_type];
    cv::putText(frame, 
        "Class "+std::to_string(armor_class)+": "+using_predictor_type_string, 
        cv::Point2f(frame.cols - 200, 50 + 30 * armor_class), 
        cv::FONT_HERSHEY_COMPLEX, 0.7, 
        cv::Scalar(0, 255, 0), 1, 8, false);
    result.predictor_type = using_predictor_type;
    result.armor_type = armor_class;
    result.pixel_horizontal_center_distance = last_pixel_horizontal_center_distance;
    result.latest_armor_distance = latest_armor_distance;

    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - latest_predicting_start_time).count()
        < pre_predict_time_not_aim) {

        result.fire_flag = false;
        result.reset = true;
        result.integrating = false;
    }

    return result;
}


RMM_fire_result_t AllPredictor::RMM_fire_control(SimpleArmor chosen_armor, RotationMotionState RMM_state, float yaw_bias, bool is_large_armor, cv::Point2d cam_to_center_vector, float choose_armor_yaw_bias_with_direction) {
    RMM_fire_result_t result;

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (RMM_fire_control_data.new_target) {
        RMM_fire_control_data.last_target_yaw_jump_time = now;
        RMM_fire_control_data.new_target = false;
    } else {
        float yaw_diff = RMM_fire_control_data.last_target_yaw - chosen_armor.yaw;
        yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));
        if (fabs(yaw_diff) > M_PI / 4.0) {
            RMM_fire_control_data.last_target_yaw_jump_time = now;
        }
    }
    RMM_fire_control_data.last_target_yaw = chosen_armor.yaw;

    if (armor_class != ArmorType::Outpost) {
        if (fabs(RMM_state.vyaw) > RMM_fire_control_data.aim_center_vyaw_upper_threshold) {
            RMM_fire_control_data.aim_center_schmitt_trigger = true;
        } else if (fabs(RMM_state.vyaw) < RMM_fire_control_data.aim_center_vyaw_lower_threshold) {
            RMM_fire_control_data.aim_center_schmitt_trigger = false;
        }
    } else {
        RMM_fire_control_data.aim_center_schmitt_trigger = false;
    }
    result.aim_center = RMM_fire_control_data.aim_center_schmitt_trigger;


    if (result.aim_center) {
        float max_yaw_bias = std::atan2((is_large_armor ? ArmorConstants::LARGE_ARMOR_WIDTH : ArmorConstants::SMALL_ARMOR_WIDTH) / 2.0, 
                                        chosen_armor.r) + RMM_fire_control_data.aim_center_yaw_bias_expand;
        if (fabs(yaw_bias) < max_yaw_bias) {
            result.fire = true;
        } else {
            result.fire = false;
        }
    } else {
        int ms_sence_last_target_change = 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - RMM_fire_control_data.last_target_yaw_jump_time).count();

        float ceasefire_armor_yaw = chosen_armor.yaw + RMM_state.vyaw * RMM_fire_control_data.before_target_change_ceasefire_ms / 1000.0;
        cv::Point2d ceasefire_armor_yaw_vector = {std::sin(ceasefire_armor_yaw + choose_armor_yaw_bias_with_direction), -std::cos(ceasefire_armor_yaw + choose_armor_yaw_bias_with_direction)};
        float ceasefire_armor_yaw_dot = cam_to_center_vector.dot(ceasefire_armor_yaw_vector);

        float jump_yaw_rad = M_PI / 2.0;
        if (armor_class == ArmorType::Outpost) {
            jump_yaw_rad = M_PI * 2.0 / 3.0;
        }

        float ceasefire_armor_yaw_1 = ceasefire_armor_yaw + jump_yaw_rad;
        cv::Point2d ceasefire_armor_yaw_vector_1 = {std::sin(ceasefire_armor_yaw_1 + choose_armor_yaw_bias_with_direction), -std::cos(ceasefire_armor_yaw_1 + choose_armor_yaw_bias_with_direction)};
        float ceasefire_armor_yaw_dot_1 = cam_to_center_vector.dot(ceasefire_armor_yaw_vector_1);

        float ceasefire_armor_yaw_2 = ceasefire_armor_yaw - jump_yaw_rad;
        cv::Point2d ceasefire_armor_yaw_vector_2 = {std::sin(ceasefire_armor_yaw_2 + choose_armor_yaw_bias_with_direction), -std::cos(ceasefire_armor_yaw_2 + choose_armor_yaw_bias_with_direction)};
        float ceasefire_armor_yaw_dot_2 = cam_to_center_vector.dot(ceasefire_armor_yaw_vector_2);

        bool before_target_change_ceasefire_flag = false;
        if (abs(RMM_state.vyaw) < RMM_fire_control_data.low_vyaw_threshold) {
            before_target_change_ceasefire_flag = false;
        } else if (ceasefire_armor_yaw_dot_1 < ceasefire_armor_yaw_dot || ceasefire_armor_yaw_dot_2 < ceasefire_armor_yaw_dot) {
            before_target_change_ceasefire_flag = true;
        }

        if (ms_sence_last_target_change < RMM_fire_control_data.after_target_change_ceasefire_ms 
            || 
            before_target_change_ceasefire_flag
        ) {
            result.fire = false;
        } else {
            result.fire = true;
        }
    }

    return result;
}
