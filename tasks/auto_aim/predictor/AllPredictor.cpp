#include "predictor/AllPredictor.h"
#include "utils/DataProcessFuncs.h"

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

std::string angleText(double radians)
{
    return std::isfinite(radians)
        ? cv::format("%.2f deg", radians * kRadToDeg)
        : std::string("N/A");
}

std::string pixelText(double pixels)
{
    return std::isfinite(pixels)
        ? cv::format("%.2f px", pixels)
        : std::string("N/A");
}

void drawYawMeasurementPanel(cv::Mat& image,
                             const YawMeasurementDebug& debug)
{
    if (image.empty() || !debug.available) return;

    const int panel_width = std::min(390, std::max(0, image.cols - 20));
    const int x = std::max(10, image.cols - panel_width - 10);
    constexpr int y = 10;
    constexpr int panel_height = 242;
    cv::rectangle(image, cv::Rect(x, y, panel_width, panel_height),
                  cv::Scalar(20, 20, 20), cv::FILLED);

    const std::vector<std::string> lines = {
        "yaw raw : " + angleText(debug.yaw_raw_rad),
        "yaw ref : " + angleText(debug.yaw_refined_rad),
        "yaw use : " + angleText(debug.yaw_used_rad),
        "dyaw    : " + angleText(debug.yaw_delta_rad),
        "repr raw: " + pixelText(debug.reprojection_rmse_raw_px),
        "repr ref: " + pixelText(debug.reprojection_rmse_refined_px),
        "facing  : " + angleText(debug.facing_angle_rad),
        "refine  : " + debug.refinement_status,
    };
    const cv::Scalar status_color = debug.refined_valid
        ? cv::Scalar(0, 255, 0)
        : (debug.refinement_status == "FALLBACK"
               ? cv::Scalar(0, 165, 255)
               : cv::Scalar(0, 255, 255));
    for (std::size_t i = 0; i < lines.size(); ++i) {
        cv::putText(image, lines[i],
                    cv::Point(x + 10, y + 27 + static_cast<int>(i) * 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.58,
                    i + 1 == lines.size() ? status_color
                                          : cv::Scalar(230, 230, 230),
                    1, cv::LINE_AA);
    }
}

void drawGeometryPanel(cv::Mat& image, const GeometryDebug& debug)
{
    if (image.empty() || !debug.available) return;

    const int panel_width = std::min(390, std::max(0, image.cols - 20));
    const int x = std::max(10, image.cols - panel_width - 10);
    constexpr int y = 260;
    constexpr int panel_height = 330;
    cv::rectangle(image, cv::Rect(x, y, panel_width, panel_height),
                  cv::Scalar(20, 20, 20), cv::FILLED);

    const std::string parity = debug.armor_parity == 0
        ? "EVEN" : (debug.armor_parity == 1 ? "ODD" : "N/A");
    std::vector<std::string> lines = {
        cv::format("geometry r1/r2/h: %.4f %.4f %.4f m",
                   debug.r1_m, debug.r2_m, debug.h_m),
        cv::format("P r1/r2/h: %.6f %.6f %.6f",
                   debug.p_r1_m2, debug.p_r2_m2, debug.p_h_m2),
        "matched: " + std::to_string(debug.matched_armor_id) +
            " parity: " + parity,
        "switch: " + std::to_string(debug.armor_switched ? 1 : 0) +
            " (SP physical armor id)",
        "geometry valid: " + std::to_string(debug.geometry_valid ? 1 : 0),
        "measurement update: " +
            std::to_string(debug.geometry_update_allowed ? 1 : 0),
        cv::format("w: %.3f rad/s  NIS: %.3f", debug.w_rad_s, debug.nis),
        "joint: " + debug.joint_status +
            cv::format(" id=%d NIS=%.2f",
                       debug.joint_second_id, debug.joint_nis),
        debug.comparison_available
            ? cv::format("A/B range J-B: %+.3f m  center: %.3f m",
                         debug.joint_minus_baseline_ground_range_m,
                         debug.center_separation_m)
            : std::string("A/B comparison: N/A"),
        "EKF: " + debug.ekf_state,
    };
    for (std::size_t i = 0; i < lines.size(); ++i) {
        cv::putText(image, lines[i],
                    cv::Point(x + 10, y + 26 + static_cast<int>(i) * 27),
                    cv::FONT_HERSHEY_SIMPLEX, 0.50,
                    debug.geometry_preserved ? cv::Scalar(255, 180, 0)
                                             : cv::Scalar(230, 230, 230),
                    1, cv::LINE_AA);
    }
}

}  // namespace

void AllPredictor::update_serial_info(float bullet_velocity, float last_pitch_rad_delayed, float last_yaw_rad_delayed, float total_yaw_rad_delayed) {
    bullet_velocity_ = bullet_velocity;
    last_pitch_rad_delayed_ = last_pitch_rad_delayed;
    last_yaw_rad_delayed_ = last_yaw_rad_delayed;
    total_yaw_rad_delayed_ = total_yaw_rad_delayed;
}

void AllPredictor::resetTarget()
{
    if (ekf_target_predictor_) {
        // 释放物理目标时同步清空滤波状态，避免新目标继承旧状态。
        ekf_target_predictor_->clear();
    }
    ekf_target_predictor_.reset();
    if (baseline_ekf_target_predictor_) {
        baseline_ekf_target_predictor_->clear();
    }
    baseline_ekf_target_predictor_.reset();
    init_r = 200.0F;
    target_active_ = false;
    has_valid_ballistic = false;
    last_total_delay_ = 0.0F;
    last_rest_frame_pos = cv::Point3f(0.0F, 0.0F, 0.0F);
    last_aim_yaw_pitch_ = cv::Point2f(0.0F, 0.0F);
    last_pixel_horizontal_center_distance = 1e10F;
    latest_armor_distance = 1e10F;
    armor_is_large = false;
    last_selected_aim_id_ = -1;

    ekf_fire_control_data.aim_center_schmitt_trigger = false;
    ekf_fire_control_data.new_target = true;
    ekf_fire_control_data.last_target_yaw = 0.0F;
    ekf_fire_control_data.last_target_yaw_jump_time =
        std::chrono::steady_clock::now();
}

void AllPredictor::startTarget()
{
    target_active_ = true;
    latest_predicting_start_time = std::chrono::steady_clock::now();
    ekf_fire_control_data.new_target = true;
}

bool AllPredictor::targetActive() const
{
    return target_active_;
}

std::optional<std::string> AllPredictor::ekfTrackerState() const
{
    if (armor_class == ArmorType::Base ||
        armor_class == ArmorType::Outpost ||
        !ekf_target_predictor_) {
        return std::nullopt;
    }
    return ekf_target_predictor_->debugState().tracker_state;
}

ArmorResult* AllPredictor::selectCurrentMeasurement(
    std::vector<ArmorResult>& candidates)
{
    ArmorResult* measurement = nullptr;
    for (ArmorResult& candidate : candidates) {
        if (measurement == nullptr ||
            (candidate.is_tracked_now && !measurement->is_tracked_now) ||
            (candidate.is_tracked_now == measurement->is_tracked_now &&
             candidate.confidence > measurement->confidence)) {
            measurement = &candidate;
        }
    }
    return measurement;
}

PredictorResult AllPredictor::step(
    std::vector<ArmorResult>& classifyResults,
    const std::vector<JointEkfTrackPair>& joint_pairs,
    cv::Mat& frame,
    double frame_timestamp_s)
{
    PredictorResult result;

    // 保存未绘制调试信息的相机画面，用于生成 EKF 投影视图。
    const cv::Mat ekf_camera_base_frame = frame.clone();

    bool ballistic_valid_flag = false;
    float total_delay = last_total_delay_;
    cv::Point3f predicted_armor_pos;
    cv::Point3f predicted_aim_pos;
    bool fire_flag = false;
    std::vector<float> cam_position = rest_frame_ -> getCamPosition();
    result.integrating = true;


    ArmorResult* current_measurement =
        selectCurrentMeasurement(classifyResults);
    ArmorResult* joint_secondary_measurement = nullptr;
    if (current_measurement != nullptr) {
        if (current_measurement->is_tracked_now) {
            for (const JointEkfTrackPair& pair : joint_pairs) {
                if (pair.number != current_measurement->number) continue;
                int secondary_track_id = -1;
                if (pair.track_id_a == current_measurement->track_id) {
                    secondary_track_id = pair.track_id_b;
                } else if (pair.track_id_b == current_measurement->track_id) {
                    secondary_track_id = pair.track_id_a;
                } else {
                    continue;
                }

                for (ArmorResult& candidate : classifyResults) {
                    if (candidate.track_id == secondary_track_id &&
                        candidate.number == pair.number &&
                        candidate.is_tracked_now) {
                        joint_secondary_measurement = &candidate;
                        break;
                    }
                }
                if (joint_secondary_measurement != nullptr) break;
            }
        }
    }

    const bool ekf_warmup_complete =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - latest_predicting_start_time)
            .count() >= pre_predict_time;
    if (current_measurement != nullptr) {
            const ArmorResult& chosen_armor = *current_measurement;
            AimResult solve_armor_result = chosen_armor.solve_armor_result;
            armor_is_large = chosen_armor.is_large;

            last_pixel_horizontal_center_distance = std::abs(chosen_armor.center.x - static_cast<float>(frame.cols)/2.0);
            latest_armor_distance = solve_armor_result.distance;
            
            // 查看z轴距离轴数据
            oscilloscope_common_ -> addDataPoint(solve_armor_result.position.z / 10000, 0);

            // 将pnp结果转换至静止坐标系以稳定预测
            cv::Point3f rest_frame_pos = rest_frame_ -> pnpToWorldP3f(solve_armor_result.position);
            std::vector<float> rest_frame_euler_angles = rest_frame_ -> getWorldEulerAnglesFromCam(
                solve_armor_result.normal_euler_angles[0], solve_armor_result.normal_euler_angles[1], solve_armor_result.normal_euler_angles[2]);

            result.yaw_debug.available = true;
            // 没有单帧偏航角优化，沿用原 RMM 的世界系偏航角。
            const double ekf_measurement_yaw =
                static_cast<double>(rest_frame_euler_angles[0]);
            result.yaw_debug.yaw_raw_rad = ekf_measurement_yaw;
            result.yaw_debug.yaw_refined_rad = ekf_measurement_yaw;
            result.yaw_debug.yaw_used_rad = ekf_measurement_yaw;
            result.yaw_debug.yaw_delta_rad = 0.0;
            result.yaw_debug.reprojection_rmse_raw_px =
                std::numeric_limits<double>::quiet_NaN();
            result.yaw_debug.reprojection_rmse_refined_px =
                std::numeric_limits<double>::quiet_NaN();
            result.yaw_debug.facing_angle_rad =
                std::numeric_limits<double>::quiet_NaN();
            result.yaw_debug.refined_valid = false;
            result.yaw_debug.refinement_status = "RM2027_WORLD_YAW";
            last_rest_frame_pos = rest_frame_pos;

            // 提前预测与弹道解算
            float bullet_time = (bullet_velocity_ > 1.0f) ? (std::abs(solve_armor_result.position.z) / 1000.0f / bullet_velocity_) : 0.0f;
            total_delay = bullet_time + extra_predict_time;
            last_total_delay_ = total_delay;

            // 基地、前哨站及 EKF 预热阶段直接使用当前观测。
            predicted_armor_pos = rest_frame_pos;
            predicted_aim_pos = predicted_armor_pos;
            fire_flag = true;

            if (!chosen_armor.is_tracked_now) {
                result.command_delta_pitch = 0.0;
                result.command_delta_yaw = 0.0;
                predicted_armor_pos = last_rest_frame_pos;
                predicted_aim_pos = last_rest_frame_pos;
            }
    } else {
        // 短暂丢失时保留滤波器并执行纯预测，进入 LOST 后才统一重置。
        result.reset = false;
        result.command_delta_pitch = 0.0;
        result.command_delta_yaw = 0.0;
        result.fire_flag = false;
        predicted_armor_pos = last_rest_frame_pos;
        predicted_aim_pos = last_rest_frame_pos;
    }





    // 普通车辆使用 SuperPower 四装甲模型，基地和前哨站保留原直接观测流程。
    if (armor_class != ArmorType::Base && armor_class != ArmorType::Outpost) {
        // ======================== SuperPower EKF ========================
        const double RMM_update_time = frame_timestamp_s;
        bool RMM_updated_flag = false;
        cv::Mat RMM_visualize_frame;
        cv::Mat EKF_vertical_frame;
        cv::Mat EKF_camera_overlay_frame;
        bool has_tracked_armor_flag = false;

        // 保存本帧输入，供 EKF 可视化使用。
        bool ekf_has_measurement = false;
        cv::Point3f ekf_measurement_world(0.0f, 0.0f, 0.0f);
        bool ekf_has_secondary_measurement = false;
        cv::Point3f ekf_secondary_measurement_world(0.0f, 0.0f, 0.0f);
        bool ekf_has_aim = false;
        cv::Point3f ekf_aim_world(0.0f, 0.0f, 0.0f);

        // 直接观测与 EKF 共用同一块主装甲板。
        if (current_measurement != nullptr) {
            AimResult solve_armor_result = current_measurement->solve_armor_result;
            cv::Point3f rest_frame_pos =
                rest_frame_->pnpToWorldP3f(solve_armor_result.position);
            std::vector<float> rest_frame_euler_angles =
                rest_frame_->getWorldEulerAnglesFromCam(
                    solve_armor_result.normal_euler_angles[0],
                    solve_armor_result.normal_euler_angles[1],
                    solve_armor_result.normal_euler_angles[2]);

            ekf_has_measurement = true;
            ekf_measurement_world = rest_frame_pos;

            EKFTargetObservation RMM_update_data({
                rest_frame_pos.x,
                rest_frame_pos.y,
                rest_frame_pos.z,
                static_cast<double>(rest_frame_euler_angles[0]),
                RMM_update_time
            });

            std::optional<EKFTargetObservation> joint_secondary_observation;
            if (joint_secondary_measurement != nullptr) {
                const AimResult& secondary_solve =
                    joint_secondary_measurement->solve_armor_result;
                ekf_secondary_measurement_world =
                    rest_frame_->pnpToWorldP3f(secondary_solve.position);
                ekf_has_secondary_measurement = true;
                const std::vector<float> secondary_world_euler =
                    rest_frame_->getWorldEulerAnglesFromCam(
                        secondary_solve.normal_euler_angles[0],
                        secondary_solve.normal_euler_angles[1],
                        secondary_solve.normal_euler_angles[2]);
                joint_secondary_observation = EKFTargetObservation{
                    ekf_secondary_measurement_world.x,
                    ekf_secondary_measurement_world.y,
                    ekf_secondary_measurement_world.z,
                    static_cast<double>(secondary_world_euler[0]),
                    RMM_update_time,
                };
            }

            bool recreate_model = !ekf_target_predictor_;
            if (ekf_target_predictor_) {
                const EKFTargetState s = ekf_target_predictor_->state();
                recreate_model =
                    !(std::isfinite(s.center_x) && std::isfinite(s.center_y) &&
                      std::isfinite(s.center_z) && std::isfinite(s.h) &&
                      std::isfinite(s.center_vx) && std::isfinite(s.center_vy) &&
                      std::isfinite(s.center_vz) && std::isfinite(s.r1) &&
                      std::isfinite(s.r2) && std::isfinite(s.yaw) &&
                      std::isfinite(s.w));
            }

            if (recreate_model) {
                ekf_target_predictor_ = std::make_shared<SuperPowerPredictor>(
                    RMM_update_data, init_r, config_file_ptr);
            } else if (joint_secondary_observation) {
                ekf_target_predictor_->updatePair(
                    RMM_update_data, *joint_secondary_observation);
            } else {
                ekf_target_predictor_->update(RMM_update_data);
            }

            // 单板基线使用相同主观测和时间戳，仅用于对比可视化。
            if (comparison_enabled_) {
                if (!baseline_ekf_target_predictor_) {
                    baseline_ekf_target_predictor_ =
                        std::make_shared<SuperPowerPredictor>(
                            RMM_update_data, init_r, config_file_ptr);
                } else {
                    baseline_ekf_target_predictor_->update(RMM_update_data);
                }
            }

            RMM_updated_flag = true;
            has_tracked_armor_flag = true;


        }

        if (!RMM_updated_flag && ekf_target_predictor_) {
            // 丢失观测时只执行状态预测，不构造伪观测。
            ekf_target_predictor_->missUpdate(RMM_update_time);
            if (comparison_enabled_ && baseline_ekf_target_predictor_) {
                baseline_ekf_target_predictor_->missUpdate(RMM_update_time);
            }
        }



        if (ekf_target_predictor_ && ekf_target_predictor_->hasState()) {
            EKFTargetPrediction RMM_pred_now_data = ekf_target_predictor_->predict(0.0);
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
            for (int RMM_pred_now_armor_i = 0; RMM_pred_now_armor_i < RMM_pred_now_data.armors.size(); RMM_pred_now_armor_i += 1) {
                EKFPredictedArmor& RMM_pred_now_armor = RMM_pred_now_data.armors[RMM_pred_now_armor_i];
                cv::Point3f RMM_pred_now_armor_p3f = rest_frame_ -> worldToPnpP3f({
                    static_cast<float>(RMM_pred_now_armor.x), 
                    static_cast<float>(RMM_pred_now_armor.y), 
                    static_cast<float>(RMM_pred_now_armor.z)
                });
                cv::Point2f RMM_pred_now_armor_pixel = armor_solver_->project3DToPixel(RMM_pred_now_armor_p3f);
                cv::circle(frame, RMM_pred_now_armor_pixel, 6, cv::Scalar(0, 255, 0), 2);
            }

            EKFTargetState RMM_state = ekf_target_predictor_->state();
            EKFTargetDebugState RMM_debug = ekf_target_predictor_->debugState();
            const bool comparison_available =
                comparison_enabled_ && baseline_ekf_target_predictor_ &&
                baseline_ekf_target_predictor_->hasState();
            EKFTargetState baseline_state;
            EKFTargetPrediction baseline_pred_now_data;
            if (comparison_available) {
                baseline_state = baseline_ekf_target_predictor_->state();
                baseline_pred_now_data =
                    baseline_ekf_target_predictor_->predict(0.0);
            }

            GeometryDebug& geometry_debug = result.geometry_debug;
            geometry_debug.available = true;
            geometry_debug.ekf_state = RMM_debug.tracker_state;
            geometry_debug.r1_m = RMM_debug.r1_m;
            geometry_debug.r2_m = RMM_debug.r2_m;
            geometry_debug.h_m = RMM_debug.h_m;
            geometry_debug.p_r1_m2 = RMM_debug.p_r1_m2;
            geometry_debug.p_r2_m2 = RMM_debug.p_r2_m2;
            geometry_debug.p_h_m2 = RMM_debug.p_h_m2;
            geometry_debug.w_rad_s = RMM_state.w;
            geometry_debug.nis = RMM_debug.nis >= 0.0
                ? RMM_debug.nis
                : std::numeric_limits<double>::quiet_NaN();
            geometry_debug.matched_armor_id = RMM_debug.matched_id;
            geometry_debug.armor_parity = RMM_debug.armor_parity;
            geometry_debug.armor_switched = RMM_debug.armor_switched;
            geometry_debug.joint_second_id = RMM_debug.joint_second_id;
            geometry_debug.joint_nis = RMM_debug.joint_nis;
            geometry_debug.joint_status = RMM_debug.joint_status;
            geometry_debug.comparison_available = comparison_available;
            if (comparison_available) {
                const double baseline_ground_range_m =
                    std::hypot(baseline_state.center_x,
                               baseline_state.center_y) / 1000.0;
                const double joint_ground_range_m =
                    std::hypot(RMM_state.center_x,
                               RMM_state.center_y) / 1000.0;
                geometry_debug.joint_minus_baseline_ground_range_m =
                    joint_ground_range_m - baseline_ground_range_m;
                geometry_debug.center_separation_m =
                    std::sqrt(
                        std::pow(RMM_state.center_x -
                                     baseline_state.center_x, 2.0) +
                        std::pow(RMM_state.center_y -
                                     baseline_state.center_y, 2.0) +
                        std::pow(RMM_state.center_z -
                                     baseline_state.center_z, 2.0)) /
                    1000.0;
            }
            geometry_debug.geometry_valid = RMM_debug.geometry_valid;
            geometry_debug.geometry_update_allowed =
                RMM_debug.geometry_update_allowed;
            geometry_debug.geometry_preserved =
                RMM_debug.geometry_preserved;
            if (ekf_warmup_complete && ekf_target_predictor_->ready()) {
                EKFTargetPrediction RMM_pred_aim_data = ekf_target_predictor_ -> predict(total_delay);
                cv::Point2d cam_to_center_vector = {RMM_pred_aim_data.center_x - cam_position[0], RMM_pred_aim_data.center_y - cam_position[1]};
                double cam_to_center_vector_norm = cv::norm(cam_to_center_vector);
                cv::Point2d unit_cam_to_center_vector = cv::Point2d(0.0, 1.0);
                if (cam_to_center_vector_norm > 1e-3) {
                    unit_cam_to_center_vector = cam_to_center_vector / cam_to_center_vector_norm;
                }
                std::vector<double> facing_losses;
                facing_losses.reserve(RMM_pred_aim_data.armors.size());
                // 方向偏置只供后续停火边界使用，不参与当前两项选板损失。
                float choose_armor_yaw_bias_with_direction = choose_armor_yaw_bias;
                choose_armor_yaw_bias_with_direction *= static_cast<float>(RMM_pred_aim_data.rotation_direction);
                for (const EKFPredictedArmor& armor : RMM_pred_aim_data.armors) {
                    facing_losses.push_back(normalizedArmorFacingLoss(
                        unit_cam_to_center_vector, armor.yaw));
                }
                // 首次选板以 EKF 的物理 matched_id 为基准，之后保持实际瞄准板 ID。
                const int previous_aim_id = last_selected_aim_id_ >= 0
                    ? last_selected_aim_id_
                    : RMM_debug.matched_id;
                int chosen_armor_id = selectArmorByFacingAndSwitchPenalty(
                    facing_losses, previous_aim_id,
                    choose_armor_switch_penalty);
                if (chosen_armor_id < 0) chosen_armor_id = 0;
                const EKFPredictedArmor chosen_armor =
                    RMM_pred_aim_data.armors[
                        static_cast<std::size_t>(chosen_armor_id)];
                last_selected_aim_id_ = chosen_armor_id;

                predicted_armor_pos = {
                    static_cast<float>(chosen_armor.x),
                    static_cast<float>(chosen_armor.y),
                    static_cast<float>(chosen_armor.z) 
                };

                const double cam_to_center_yaw = std::atan2(
                    -(RMM_pred_aim_data.center_x - cam_position[0]),
                    RMM_pred_aim_data.center_y - cam_position[1]);
                float chosen_armor_yaw_bias =
                    (chosen_armor.yaw - cam_to_center_yaw) *
                    static_cast<float>(RMM_pred_aim_data.rotation_direction);
                chosen_armor_yaw_bias = atan2(sin(chosen_armor_yaw_bias), cos(chosen_armor_yaw_bias));

                EKF_fire_result_t RMM_fire_result = EKF_fire_control(chosen_armor, RMM_state, chosen_armor_yaw_bias, armor_is_large, cam_to_center_vector, choose_armor_yaw_bias_with_direction);
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
                // 保持瞄准连续，但装甲切换或滤波未稳定时禁止开火。
                if (!ekf_target_predictor_->ready() ||
                    RMM_debug.armor_switched) {
                    fire_flag = false;
                }

                ekf_has_aim = true;
                ekf_aim_world = predicted_aim_pos;
                
            }
            // EKF 顶视图、纵向视图与相机投影视图。
            {
                constexpr int kTopSize = 900;
                constexpr int kVerticalWidth = 900;
                constexpr int kVerticalHeight = 260;
                constexpr double kTopScalePxPerMm = 0.110;       // 110 px/m
                constexpr double kVerticalYScalePxPerMm = 0.080; // 80 px/m
                constexpr double kVerticalZScalePxPerMm = 0.160; // 160 px/m

                RMM_visualize_frame =
                    cv::Mat(kTopSize, kTopSize, CV_8UC3,
                            cv::Scalar(248, 248, 248));
                EKF_vertical_frame =
                    cv::Mat(kVerticalHeight, kVerticalWidth, CV_8UC3,
                            cv::Scalar(248, 248, 248));

                const int top_c = kTopSize / 2;
                auto world_to_top = [&](double x_mm, double y_mm) {
                    return cv::Point(
                        static_cast<int>(top_c + x_mm * kTopScalePxPerMm),
                        static_cast<int>(top_c - y_mm * kTopScalePxPerMm));
                };
                auto world_to_vertical = [&](double y_mm, double z_mm) {
                    constexpr int x0 = 80;
                    constexpr int y0 = kVerticalHeight - 40;
                    return cv::Point(
                        static_cast<int>(x0 + y_mm * kVerticalYScalePxPerMm),
                        static_cast<int>(y0 - z_mm * kVerticalZScalePxPerMm));
                };

                cv::line(RMM_visualize_frame, cv::Point(0, top_c),
                         cv::Point(kTopSize, top_c),
                         cv::Scalar(220, 220, 220), 1);
                cv::line(RMM_visualize_frame, cv::Point(top_c, 0),
                         cv::Point(top_c, kTopSize),
                         cv::Scalar(220, 220, 220), 1);

                // 世界坐标原点作为相机参考原点。
                const cv::Point camera_p(top_c, top_c);
                cv::circle(RMM_visualize_frame, camera_p, 6,
                           cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
                cv::putText(RMM_visualize_frame, "camera",
                            camera_p + cv::Point(8, 18),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45,
                            cv::Scalar(20, 20, 20), 1, cv::LINE_AA);

                if (ekf_has_measurement) {
                    const cv::Point p =
                        world_to_top(ekf_measurement_world.x,
                                     ekf_measurement_world.y);
                    cv::circle(RMM_visualize_frame, p, 12,
                               cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
                    cv::putText(RMM_visualize_frame, "PnP-A",
                                p + cv::Point(10, -10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                                cv::Scalar(180, 0, 180), 1, cv::LINE_AA);
                }
                if (RMM_debug.joint_pair_used &&
                    ekf_has_secondary_measurement) {
                    const cv::Point p = world_to_top(
                        ekf_secondary_measurement_world.x,
                        ekf_secondary_measurement_world.y);
                    cv::circle(RMM_visualize_frame, p, 12,
                               cv::Scalar(0, 140, 255), 2, cv::LINE_AA);
                    cv::putText(RMM_visualize_frame, "PnP-B",
                                p + cv::Point(10, 18),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                                cv::Scalar(0, 110, 220), 1, cv::LINE_AA);
                }

                const cv::Point center_p =
                    world_to_top(RMM_pred_now_data.center_x,
                                 RMM_pred_now_data.center_y);
                cv::circle(RMM_visualize_frame, center_p, 6,
                           cv::Scalar(0, 125, 0), -1, cv::LINE_AA);
                cv::putText(
                    RMM_visualize_frame,
                    comparison_available ? "JC" : "EC",
                    center_p + cv::Point(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(0, 125, 0), 1, cv::LINE_AA);

                if (comparison_available) {
                    const cv::Point baseline_center_p =
                        world_to_top(baseline_pred_now_data.center_x,
                                     baseline_pred_now_data.center_y);
                    cv::drawMarker(
                        RMM_visualize_frame, baseline_center_p,
                        cv::Scalar(255, 0, 0), cv::MARKER_TILTED_CROSS,
                        18, 2, cv::LINE_AA);
                    cv::putText(
                        RMM_visualize_frame, "BC",
                        baseline_center_p + cv::Point(8, 16),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45,
                        cv::Scalar(200, 0, 0), 1, cv::LINE_AA);
                    cv::line(RMM_visualize_frame, baseline_center_p, center_p,
                             cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
                }

                for (int i = 0;
                     i < static_cast<int>(RMM_pred_now_data.armors.size()); ++i) {
                    const auto& armor = RMM_pred_now_data.armors[i];
                    const cv::Point p = world_to_top(armor.x, armor.y);
                    const bool primary_used = (i == RMM_debug.matched_id);
                    const bool secondary_used =
                        RMM_debug.joint_pair_used &&
                        (i == RMM_debug.joint_second_id);
                    const bool pnp_used = primary_used || secondary_used;
                    const cv::Scalar color =
                        primary_used
                            ? cv::Scalar(0, 205, 0)
                            : (secondary_used ? cv::Scalar(0, 140, 255)
                                              : cv::Scalar(0, 125, 0));
                    const std::string label =
                        "E" + std::to_string(i) +
                        (primary_used ? " PnP-A"
                                      : (secondary_used ? " PnP-B" : ""));

                    cv::circle(RMM_visualize_frame, p,
                               pnp_used ? 9 : 6, color,
                               pnp_used ? -1 : 2, cv::LINE_AA);
                    cv::putText(RMM_visualize_frame,
                                label,
                                p + cv::Point(8, 14),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                                color, 1, cv::LINE_AA);

                    const cv::Point arrow_end(
                        static_cast<int>(p.x + 26 * std::sin(armor.yaw)),
                        static_cast<int>(p.y + 26 * std::cos(armor.yaw)));
                    cv::arrowedLine(RMM_visualize_frame, p, arrow_end,
                                    color, 1, cv::LINE_AA, 0, 0.25);
                }

                if (ekf_has_aim) {
                    const cv::Point aim_p =
                        world_to_top(ekf_aim_world.x, ekf_aim_world.y);
                    cv::line(RMM_visualize_frame, camera_p, aim_p,
                             cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                    cv::drawMarker(RMM_visualize_frame, aim_p,
                                   cv::Scalar(0, 0, 255),
                                   cv::MARKER_CROSS, 26, 2, cv::LINE_AA);
                    cv::circle(RMM_visualize_frame, aim_p, 11,
                               cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                    cv::putText(RMM_visualize_frame, "aim",
                                aim_p + cv::Point(12, -12),
                                cv::FONT_HERSHEY_SIMPLEX, 0.48,
                                cv::Scalar(0, 0, 200), 1, cv::LINE_AA);
                }

                std::string ekf_title = cv::format(
                    "SUPERPOWER TARGET EKF  t=%.3fs  dt=%.1fms",
                    RMM_update_time, RMM_debug.dt_s * 1000.0);
                if (RMM_debug.time_discontinuity) {
                    ekf_title += "  TIME RESET";
                }
                cv::putText(
                    RMM_visualize_frame, ekf_title,
                    cv::Point(14, 24), cv::FONT_HERSHEY_SIMPLEX, 0.56,
                    RMM_debug.time_discontinuity
                        ? cv::Scalar(0, 0, 220)
                        : cv::Scalar(30, 30, 30),
                    1, cv::LINE_AA);

                cv::putText(
                    RMM_visualize_frame,
                    cv::format(
                        "center=(%.3f,%.3f,%.3f)m EKF_yaw=%.1fdeg w=%.2frad/s",
                        RMM_state.center_x / 1000.0,
                        RMM_state.center_y / 1000.0,
                        RMM_state.center_z / 1000.0,
                        RMM_state.yaw * 180.0 / M_PI,
                        RMM_state.w),
                    cv::Point(14, 49), cv::FONT_HERSHEY_SIMPLEX, 0.50,
                    cv::Scalar(50, 50, 50), 1, cv::LINE_AA);

                cv::putText(
                    RMM_visualize_frame,
                    result.yaw_debug.available
                        ? cv::format("measurement_yaw=%.1fdeg",
                                     result.yaw_debug.yaw_used_rad * kRadToDeg)
                        : std::string("measurement_yaw=NA"),
                    cv::Point(14, 144), cv::FONT_HERSHEY_SIMPLEX, 0.50,
                    cv::Scalar(100, 40, 120), 1, cv::LINE_AA);

                const cv::Scalar tracker_color =
                    RMM_debug.tracker_state == "TRACKING"
                        ? cv::Scalar(0, 120, 0)
                        : (RMM_debug.tracker_state == "TEMP_LOST"
                               ? cv::Scalar(0, 140, 220)
                               : cv::Scalar(0, 0, 180));
                cv::putText(
                    RMM_visualize_frame,
                    "tracker=" + RMM_debug.tracker_state +
                        " matched=" + std::to_string(RMM_debug.matched_id) +
                        " update=" + std::to_string(RMM_debug.updated ? 1 : 0) +
                        " lost=" + std::to_string(RMM_debug.lost_frames),
                    cv::Point(14, 75), cv::FONT_HERSHEY_SIMPLEX, 0.53,
                    tracker_color, 1, cv::LINE_AA);

                std::string residual_line =
                    cv::format("NIS=%.2f pos_err=%.3fm yaw_err=%.1fdeg",
                               RMM_debug.nis,
                               RMM_debug.position_error_m,
                               RMM_debug.yaw_error_deg);
                if (RMM_debug.armor_switched)
                    residual_line += "  ARMOR_SWITCH";
                cv::putText(RMM_visualize_frame, residual_line,
                            cv::Point(14, 98),
                            cv::FONT_HERSHEY_SIMPLEX, 0.50,
                            cv::Scalar(80, 50, 20), 1, cv::LINE_AA);

                std::string association_line =
                    "SP association: nearest 3 by range, min(angle+bearing)";
                if (RMM_debug.armor_switched)
                    association_line += "  SWITCH";
                cv::putText(
                    RMM_visualize_frame, association_line,
                    cv::Point(14, 121), cv::FONT_HERSHEY_SIMPLEX, 0.50,
                    cv::Scalar(90, 70, 20),
                    1, cv::LINE_AA);

                const int baseline = kVerticalHeight - 40;
                cv::line(EKF_vertical_frame, cv::Point(15, baseline),
                         cv::Point(kVerticalWidth - 15, baseline),
                         cv::Scalar(180, 180, 180), 1);
                cv::putText(EKF_vertical_frame,
                            "forward y ->    vertical z ^",
                            cv::Point(15, 24),
                            cv::FONT_HERSHEY_SIMPLEX, 0.48,
                            cv::Scalar(60, 60, 60), 1, cv::LINE_AA);

                if (ekf_has_measurement) {
                    const cv::Point p =
                        world_to_vertical(ekf_measurement_world.y,
                                          ekf_measurement_world.z);
                    cv::circle(EKF_vertical_frame, p, 8,
                               cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
                    cv::putText(EKF_vertical_frame, "PnP-A",
                                p + cv::Point(8, -8),
                                cv::FONT_HERSHEY_SIMPLEX, 0.42,
                                cv::Scalar(180, 0, 180), 1, cv::LINE_AA);
                }
                if (RMM_debug.joint_pair_used &&
                    ekf_has_secondary_measurement) {
                    const cv::Point p = world_to_vertical(
                        ekf_secondary_measurement_world.y,
                        ekf_secondary_measurement_world.z);
                    cv::circle(EKF_vertical_frame, p, 8,
                               cv::Scalar(0, 140, 255), 2, cv::LINE_AA);
                    cv::putText(EKF_vertical_frame, "PnP-B",
                                p + cv::Point(8, 16),
                                cv::FONT_HERSHEY_SIMPLEX, 0.42,
                                cv::Scalar(0, 110, 220), 1, cv::LINE_AA);
                }

                for (int i = 0;
                     i < static_cast<int>(RMM_pred_now_data.armors.size()); ++i) {
                    const auto& armor = RMM_pred_now_data.armors[i];
                    const bool primary_used = (i == RMM_debug.matched_id);
                    const bool secondary_used =
                        RMM_debug.joint_pair_used &&
                        (i == RMM_debug.joint_second_id);
                    const bool pnp_used = primary_used || secondary_used;
                    const cv::Point p =
                        world_to_vertical(armor.y, armor.z);
                    cv::drawMarker(
                        EKF_vertical_frame, p,
                        primary_used
                            ? cv::Scalar(0, 190, 0)
                            : (secondary_used ? cv::Scalar(0, 140, 255)
                                              : cv::Scalar(0, 120, 0)),
                        cv::MARKER_CROSS,
                        pnp_used ? 16 : 11,
                        2, cv::LINE_AA);
                    if (pnp_used) {
                        cv::putText(
                            EKF_vertical_frame,
                            primary_used ? "PnP-A" : "PnP-B",
                            p + cv::Point(7, -7),
                            cv::FONT_HERSHEY_SIMPLEX, 0.38,
                            primary_used ? cv::Scalar(0, 190, 0)
                                         : cv::Scalar(0, 110, 220),
                            1, cv::LINE_AA);
                    }
                }

                if (ekf_has_aim) {
                    const cv::Point aim_v =
                        world_to_vertical(ekf_aim_world.y, ekf_aim_world.z);
                    cv::drawMarker(EKF_vertical_frame, aim_v,
                                   cv::Scalar(0, 0, 255),
                                   cv::MARKER_CROSS, 22, 2, cv::LINE_AA);
                    cv::circle(EKF_vertical_frame, aim_v, 9,
                               cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                }

                // 将当前 EKF 几何模型投影回真实相机画面。
                EKF_camera_overlay_frame = ekf_camera_base_frame.clone();

                auto project_world_to_camera = [&](const cv::Point3f& world_mm,
                                                   cv::Point2f& pixel) -> bool {
                    const cv::Point3f pnp = rest_frame_->worldToPnpP3f(world_mm);
                    if (!std::isfinite(pnp.x) || !std::isfinite(pnp.y) ||
                        !std::isfinite(pnp.z) || pnp.z <= 1.0f) {
                        return false;
                    }
                    pixel = armor_solver_->project3DToPixel(pnp);
                    return std::isfinite(pixel.x) && std::isfinite(pixel.y) &&
                           pixel.x >= 0.0f &&
                           pixel.x < static_cast<float>(EKF_camera_overlay_frame.cols) &&
                           pixel.y >= 0.0f &&
                           pixel.y < static_cast<float>(EKF_camera_overlay_frame.rows);
                };

                cv::Point2f center_px;
                const bool center_visible = project_world_to_camera(
                    cv::Point3f(static_cast<float>(RMM_pred_now_data.center_x),
                                static_cast<float>(RMM_pred_now_data.center_y),
                                static_cast<float>(RMM_pred_now_data.center_z)),
                    center_px);
                if (center_visible) {
                    cv::drawMarker(EKF_camera_overlay_frame, center_px,
                                   cv::Scalar(0, 180, 0),
                                   cv::MARKER_CROSS, 20, 2, cv::LINE_AA);
                    cv::putText(
                        EKF_camera_overlay_frame,
                        comparison_available ? "JC" : "EC",
                        center_px + cv::Point2f(8.0f, -8.0f),
                        cv::FONT_HERSHEY_SIMPLEX, 0.48,
                        cv::Scalar(0, 180, 0), 1, cv::LINE_AA);
                }
                if (comparison_available) {
                    cv::Point2f baseline_center_px;
                    if (project_world_to_camera(
                            cv::Point3f(
                                static_cast<float>(
                                    baseline_pred_now_data.center_x),
                                static_cast<float>(
                                    baseline_pred_now_data.center_y),
                                static_cast<float>(
                                    baseline_pred_now_data.center_z)),
                            baseline_center_px)) {
                        cv::drawMarker(
                            EKF_camera_overlay_frame, baseline_center_px,
                            cv::Scalar(255, 0, 0),
                            cv::MARKER_TILTED_CROSS, 18, 2, cv::LINE_AA);
                        cv::putText(
                            EKF_camera_overlay_frame, "BC",
                            baseline_center_px + cv::Point2f(8.0f, 16.0f),
                            cv::FONT_HERSHEY_SIMPLEX, 0.46,
                            cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
                    }
                }

                for (int i = 0;
                     i < static_cast<int>(RMM_pred_now_data.armors.size()); ++i) {
                    const auto& armor = RMM_pred_now_data.armors[i];
                    cv::Point2f armor_px;
                    if (!project_world_to_camera(
                            cv::Point3f(static_cast<float>(armor.x),
                                        static_cast<float>(armor.y),
                                        static_cast<float>(armor.z)),
                            armor_px)) {
                        continue;
                    }

                    const bool primary_used = (i == RMM_debug.matched_id);
                    const bool secondary_used =
                        RMM_debug.joint_pair_used &&
                        (i == RMM_debug.joint_second_id);
                    const bool pnp_used = primary_used || secondary_used;
                    const cv::Scalar armor_color =
                        primary_used
                            ? cv::Scalar(0, 255, 0)
                            : (secondary_used ? cv::Scalar(0, 165, 255)
                                              : cv::Scalar(0, 170, 0));
                    const std::string armor_label =
                        "E" + std::to_string(i) +
                        (primary_used ? " PnP-A"
                                      : (secondary_used ? " PnP-B" : ""));

                    if (center_visible) {
                        cv::line(EKF_camera_overlay_frame, center_px, armor_px,
                                 cv::Scalar(80, 120, 80), 1, cv::LINE_AA);
                    }
                    cv::circle(EKF_camera_overlay_frame, armor_px,
                               pnp_used ? 9 : 6, armor_color,
                               pnp_used ? -1 : 2, cv::LINE_AA);
                    cv::putText(EKF_camera_overlay_frame,
                                armor_label,
                                armor_px + cv::Point2f(8.0f, -8.0f),
                                cv::FONT_HERSHEY_SIMPLEX, 0.48,
                                armor_color, 1, cv::LINE_AA);
                }

                // 只标记本帧实际进入 EKF 更新的观测。
                if (ekf_has_measurement) {
                    cv::Point2f measurement_px;
                    if (project_world_to_camera(ekf_measurement_world,
                                                measurement_px)) {
                        cv::circle(EKF_camera_overlay_frame, measurement_px, 7,
                                   cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
                        cv::putText(EKF_camera_overlay_frame, "PnP-A",
                                    measurement_px + cv::Point2f(8.0f, 16.0f),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.42,
                                    cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
                    }
                }
                if (RMM_debug.joint_pair_used &&
                    ekf_has_secondary_measurement) {
                    cv::Point2f measurement_px;
                    if (project_world_to_camera(
                            ekf_secondary_measurement_world,
                            measurement_px)) {
                        cv::circle(EKF_camera_overlay_frame, measurement_px, 7,
                                   cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
                        cv::putText(EKF_camera_overlay_frame, "PnP-B",
                                    measurement_px + cv::Point2f(8.0f, 16.0f),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.42,
                                    cv::Scalar(0, 165, 255), 1, cv::LINE_AA);
                    }
                }

                if (ekf_has_aim) {
                    cv::Point2f aim_px;
                    if (project_world_to_camera(ekf_aim_world, aim_px)) {
                        cv::drawMarker(EKF_camera_overlay_frame, aim_px,
                                       cv::Scalar(0, 0, 255),
                                       cv::MARKER_CROSS, 26, 2, cv::LINE_AA);
                        cv::circle(EKF_camera_overlay_frame, aim_px, 11,
                                   cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                        cv::putText(EKF_camera_overlay_frame, "AIM",
                                    aim_px + cv::Point2f(12.0f, -12.0f),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.52,
                                    cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                    }
                }

                cv::putText(
                    EKF_camera_overlay_frame,
                    "EKF " + RMM_debug.tracker_state +
                        "  matched=" + std::to_string(RMM_debug.matched_id) +
                        cv::format("  meas_yaw=%.1f  ekf_yaw=%.1f  w=%.2f  NIS=%.2f",
                                   result.yaw_debug.yaw_used_rad * kRadToDeg,
                                   RMM_state.yaw * kRadToDeg,
                                   RMM_state.w, RMM_debug.nis),
                    cv::Point(14, 28), cv::FONT_HERSHEY_SIMPLEX, 0.58,
                    cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

                drawYawMeasurementPanel(EKF_camera_overlay_frame,
                                        result.yaw_debug);
                drawGeometryPanel(EKF_camera_overlay_frame,
                                  result.geometry_debug);

                result.info_images.RMM_visualize_frame = RMM_visualize_frame;
                result.info_images.EKF_vertical_frame = EKF_vertical_frame;
                result.info_images.EKF_camera_overlay_frame =
                    EKF_camera_overlay_frame;
            }
        }
        // ===================== SuperPower EKF 结束 =====================
    }

    drawYawMeasurementPanel(frame, result.yaw_debug);
    drawGeometryPanel(frame, result.geometry_debug);

    // 统一转换回pnp相机坐标系    
    predicted_aim_pos = rest_frame_ -> worldToPnpP3f(predicted_aim_pos);
    predicted_armor_pos = rest_frame_ -> worldToPnpP3f(predicted_armor_pos);

    // 弹道解算
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
        
        // 发布云台控制命令
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
    
    oscilloscope_common_ -> addDataPoint(((float)(result.fire_flag))/11.0, 1);
    oscilloscope_common_ -> update();
    result.info_images.common_debug_oscilloscope_frame = oscilloscope_common_ -> getDisplay();

    const std::string predictor_status =
        (armor_class == ArmorType::Base || armor_class == ArmorType::Outpost)
            ? "DIRECT"
            : "SP-EKF";
    cv::putText(frame, 
        "Class "+std::to_string(armor_class)+": "+predictor_status,
        cv::Point2f(frame.cols - 200, 50 + 30 * armor_class), 
        cv::FONT_HERSHEY_COMPLEX, 0.7, 
        cv::Scalar(0, 255, 0), 1, 8, false);
    result.armor_type = armor_class;
    result.predictor_type =
        (armor_class == ArmorType::Base || armor_class == ArmorType::Outpost)
            ? PredictorType::None
            : PredictorType::SuperPowerEKF;
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


EKF_fire_result_t AllPredictor::EKF_fire_control(EKFPredictedArmor chosen_armor, EKFTargetState ekf_state, float yaw_bias, bool is_large_armor, cv::Point2d cam_to_center_vector, float choose_armor_yaw_bias_with_direction) {
    EKF_fire_result_t result{};

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (ekf_fire_control_data.new_target) {
        ekf_fire_control_data.last_target_yaw_jump_time = now;
        ekf_fire_control_data.new_target = false;
    } else {
        float yaw_diff = ekf_fire_control_data.last_target_yaw - chosen_armor.yaw;
        yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff));
        if (fabs(yaw_diff) > M_PI / 4.0) {
            ekf_fire_control_data.last_target_yaw_jump_time = now;
        }
    }
    ekf_fire_control_data.last_target_yaw = chosen_armor.yaw;

    if (fabs(ekf_state.w) > ekf_fire_control_data.aim_center_vyaw_upper_threshold) {
        ekf_fire_control_data.aim_center_schmitt_trigger = true;
    } else if (fabs(ekf_state.w) < ekf_fire_control_data.aim_center_vyaw_lower_threshold) {
        ekf_fire_control_data.aim_center_schmitt_trigger = false;
    }
    result.aim_center = ekf_fire_control_data.aim_center_schmitt_trigger;


    if (result.aim_center) {
        float max_yaw_bias = std::atan2((is_large_armor ? ArmorConstants::LARGE_ARMOR_WIDTH : ArmorConstants::SMALL_ARMOR_WIDTH) / 2.0, 
                                        chosen_armor.r) + ekf_fire_control_data.aim_center_yaw_bias_expand;
        if (fabs(yaw_bias) < max_yaw_bias) {
            result.fire = true;
        } else {
            result.fire = false;
        }
    } else {
        int ms_sence_last_target_change = 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - ekf_fire_control_data.last_target_yaw_jump_time).count();

        float ceasefire_armor_yaw = chosen_armor.yaw + ekf_state.w * ekf_fire_control_data.before_target_change_ceasefire_ms / 1000.0;
        cv::Point2d ceasefire_armor_yaw_vector = {std::sin(ceasefire_armor_yaw + choose_armor_yaw_bias_with_direction), -std::cos(ceasefire_armor_yaw + choose_armor_yaw_bias_with_direction)};
        float ceasefire_armor_yaw_dot = cam_to_center_vector.dot(ceasefire_armor_yaw_vector);

        constexpr float jump_yaw_rad = M_PI / 2.0;

        float ceasefire_armor_yaw_1 = ceasefire_armor_yaw + jump_yaw_rad;
        cv::Point2d ceasefire_armor_yaw_vector_1 = {std::sin(ceasefire_armor_yaw_1 + choose_armor_yaw_bias_with_direction), -std::cos(ceasefire_armor_yaw_1 + choose_armor_yaw_bias_with_direction)};
        float ceasefire_armor_yaw_dot_1 = cam_to_center_vector.dot(ceasefire_armor_yaw_vector_1);

        float ceasefire_armor_yaw_2 = ceasefire_armor_yaw - jump_yaw_rad;
        cv::Point2d ceasefire_armor_yaw_vector_2 = {std::sin(ceasefire_armor_yaw_2 + choose_armor_yaw_bias_with_direction), -std::cos(ceasefire_armor_yaw_2 + choose_armor_yaw_bias_with_direction)};
        float ceasefire_armor_yaw_dot_2 = cam_to_center_vector.dot(ceasefire_armor_yaw_vector_2);

        bool before_target_change_ceasefire_flag = false;
        if (abs(ekf_state.w) < ekf_fire_control_data.low_vyaw_threshold) {
            before_target_change_ceasefire_flag = false;
        } else if (ceasefire_armor_yaw_dot_1 < ceasefire_armor_yaw_dot || ceasefire_armor_yaw_dot_2 < ceasefire_armor_yaw_dot) {
            before_target_change_ceasefire_flag = true;
        }

        if (ms_sence_last_target_change < ekf_fire_control_data.after_target_change_ceasefire_ms
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
