#include "predictor/AllPredictor.h"
#include "utils/DataProcessFuncs.h"

#ifndef RCLCPP_DEBUG
// rm2027 is ROS-free. The validated predictor only used RCLCPP_DEBUG for
// diagnostic printing, so discard those calls without evaluating their args.
#define RCLCPP_DEBUG(...) do {} while (0)
#endif

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
        // TargetManager release is a hard physical-target boundary. Clear the
        // SuperPower Target so a new vehicle cannot inherit its EKF state.
        ekf_target_predictor_->clear();
    }
    ekf_target_predictor_.reset();
    init_r = 200.0F;
    target_active_ = false;
    has_valid_ballistic = false;
    last_total_delay_ = 0.0F;
    last_rest_frame_pos = cv::Point3f(0.0F, 0.0F, 0.0F);
    last_aim_yaw_pitch_ = cv::Point2f(0.0F, 0.0F);
    last_pixel_horizontal_center_distance = 1e10F;
    latest_armor_distance = 1e10F;
    armor_is_large = false;

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

PredictorResult AllPredictor::step(std::vector<ArmorResult>& classifyResults,
                                   cv::Mat& frame,
                                   double frame_timestamp_s)
{
    PredictorResult result;

    // Preserve the real camera image before predictor/debug drawing mutates `frame`.
    // The dedicated EKF camera window is built only from this image plus SuperPower-EKF data.
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
    if (current_measurement != nullptr) {
        result.has_measurement = true;
        result.measurement_number = current_measurement->number;
        result.measurement_center = current_measurement->center;
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
            // std::vector<float> rest_frame_euler_angles = {
            //     static_cast<float>(solve_armor_result.ba_global_ypr[0]),
            //     static_cast<float>(solve_armor_result.ba_global_ypr[1]),
            //     static_cast<float>(solve_armor_result.ba_global_ypr[2])
            // };
            std::vector<float> rest_frame_euler_angles = rest_frame_ -> getWorldEulerAnglesFromCam(
                solve_armor_result.normal_euler_angles[0], solve_armor_result.normal_euler_angles[1], solve_armor_result.normal_euler_angles[2]);

            result.yaw_debug.available = true;
            result.yaw_debug.target_type = static_cast<int>(armor_class);
            result.yaw_debug.measurement_number = chosen_armor.number;
            result.yaw_debug.measurement_world_mm = rest_frame_pos;
            // rm2027 currently has no single-frame yaw-refinement fields.
            // Use the same world-yaw path already used by its legacy RMM:
            // camera-frame normal Euler -> RestFrame world yaw.
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
            RCLCPP_DEBUG(node->get_logger(), "camera euler angles: yaw=%.2f, pitch=%.2f, roll=%.2f", solve_armor_result.normal_euler_angles[0], solve_armor_result.normal_euler_angles[1], solve_armor_result.normal_euler_angles[2]);
            RCLCPP_DEBUG(node->get_logger(), "Rest frame pos: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", rest_frame_pos.x, rest_frame_pos.y, rest_frame_pos.z, rest_frame_euler_angles[0]);

            last_rest_frame_pos = rest_frame_pos;

            // 提前预测与弹道解算
            float bullet_time = (bullet_velocity_ > 1.0f) ? (std::abs(solve_armor_result.position.z) / 1000.0f / bullet_velocity_) : 0.0f;
            total_delay = bullet_time + extra_predict_time;
            last_total_delay_ = total_delay;

            // Direct observation is the safe fallback for Base, Outpost and EKF warm-up.
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
        // TargetManager owns the target lifetime. During TEMP_LOST an empty
        // candidate list must keep this predictor alive so the EKF can execute
        // missUpdate() below; resetTarget() is called only on a state transition
        // to LOST.
        result.reset = false;
        result.command_delta_pitch = 0.0;
        result.command_delta_yaw = 0.0;
        result.fire_flag = false;
        predicted_armor_pos = last_rest_frame_pos;
        predicted_aim_pos = last_rest_frame_pos;
    }





    // The copied SuperPower normal branch is the four-armor vehicle model.
    // Outpost/Base remain on their existing direct-observation path because this
    // project interface does not yet provide SuperPower ArmorName/ArmorType semantics.
    if (armor_class != ArmorType::Base && armor_class != ArmorType::Outpost) {
        // ======================== SuperPower EKF ========================
        const double RMM_update_time = frame_timestamp_s;
        bool RMM_updated_flag = false;
        // Scratch canvas is rebuilt below as the pure SuperPower-EKF top view.
        cv::Mat RMM_visualize_frame = cv::Mat::zeros(800, 800, CV_8UC3);
        cv::Mat EKF_vertical_frame;
        cv::Mat EKF_camera_overlay_frame;
        bool has_tracked_armor_flag = false;

        // Inputs retained only for the pure EKF debug views.
        bool ekf_has_measurement = false;
        cv::Point3f ekf_measurement_world(0.0f, 0.0f, 0.0f);
        bool ekf_has_aim = false;
        cv::Point3f ekf_aim_world(0.0f, 0.0f, 0.0f);

        // Direct observation and EKF update deliberately share the one
        // tracked-first/confidence-second measurement selected above.
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
            } else {
                ekf_target_predictor_->update(RMM_update_data);
            }

            RMM_updated_flag = true;
            has_tracked_armor_flag = true;

            cv::circle(
                RMM_visualize_frame,
                cv::Point2f(
                    400 + rest_frame_pos.x / RMM_visualize_zoom_out_factor,
                    400 - rest_frame_pos.y / RMM_visualize_zoom_out_factor),
                8, cv::Scalar(255, 255, 0), 2);

            cv::line(
                RMM_visualize_frame,
                cv::Point2f(
                    400 + rest_frame_pos.x / RMM_visualize_zoom_out_factor,
                    400 - rest_frame_pos.y / RMM_visualize_zoom_out_factor),
                cv::Point2f(
                    400 + rest_frame_pos.x / RMM_visualize_zoom_out_factor +
                        std::sin(rest_frame_euler_angles[0]) * 500 /
                            RMM_visualize_zoom_out_factor,
                    400 - (rest_frame_pos.y / RMM_visualize_zoom_out_factor -
                        std::cos(rest_frame_euler_angles[0]) * 500 /
                            RMM_visualize_zoom_out_factor)),
                cv::Scalar(255, 255, 0), 2);

        }

        if (!RMM_updated_flag && ekf_target_predictor_) {
            // SuperPower tracker performs predict/TEMP_LOST here; no pseudo-measurement update.
            ekf_target_predictor_->missUpdate(RMM_update_time);
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
            if (has_tracked_armor_flag) {
                cv::circle(RMM_visualize_frame, cv::Point2f(400+RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor), 8, cv::Scalar(0, 255, 0), 2);
            } else {
                cv::circle(RMM_visualize_frame, cv::Point2f(400+RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor), 8, cv::Scalar(255, 0, 255), 2);
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
                // cv::line(frame, RMM_pred_now_center_pixel, RMM_pred_now_armor_pixel, cv::Scalar(0, 255, 0), 2);
                
                cv::circle(RMM_visualize_frame, cv::Point2f(400+RMM_pred_now_armor.x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_armor.y/RMM_visualize_zoom_out_factor), 8, 
                    cv::Scalar(255 - RMM_pred_now_armor_i * 60, 0, RMM_pred_now_armor_i * 60), 2);
                // cv::line(RMM_visualize_frame, 
                //     cv::Point2f(400+RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor), 
                //     cv::Point2f(400+RMM_pred_now_armor.x/RMM_visualize_zoom_out_factor, 400-RMM_pred_now_armor.y/RMM_visualize_zoom_out_factor), 
                //     cv::Scalar(255 - RMM_pred_now_armor_i * 60, 0, RMM_pred_now_armor_i * 60), 2);
            }

            EKFTargetState RMM_state = ekf_target_predictor_->state();
            EKFTargetDebugState RMM_debug = ekf_target_predictor_->debugState();
            result.geometry_debug.available = true;
            result.geometry_debug.target_type = static_cast<int>(armor_class);
            result.geometry_debug.measurement_number =
                current_measurement != nullptr ? current_measurement->number : -1;
            result.geometry_debug.ekf_state = RMM_debug.tracker_state;
            result.geometry_debug.tracker_state_before =
                RMM_debug.tracker_state_before;
            result.geometry_debug.dt_s = RMM_debug.dt_s;
            result.geometry_debug.r1_m = RMM_debug.r1_m;
            result.geometry_debug.r2_m = RMM_debug.r2_m;
            result.geometry_debug.h_m = RMM_debug.h_m;
            result.geometry_debug.p_r1_m2 = RMM_debug.p_r1_m2;
            result.geometry_debug.p_r2_m2 = RMM_debug.p_r2_m2;
            result.geometry_debug.p_h_m2 = RMM_debug.p_h_m2;
            result.geometry_debug.center_x_m = RMM_state.center_x / 1000.0;
            result.geometry_debug.center_y_m = RMM_state.center_y / 1000.0;
            result.geometry_debug.center_z_m = RMM_state.center_z / 1000.0;
            result.geometry_debug.vx_m_s = RMM_state.center_vx / 1000.0;
            result.geometry_debug.vy_m_s = RMM_state.center_vy / 1000.0;
            result.geometry_debug.vz_m_s = RMM_state.center_vz / 1000.0;
            result.geometry_debug.state_yaw_rad = RMM_state.yaw;
            result.geometry_debug.w_rad_s = RMM_state.w;
            result.geometry_debug.nis = RMM_debug.nis >= 0.0
                ? RMM_debug.nis
                : std::numeric_limits<double>::quiet_NaN();
            result.geometry_debug.matched_armor_id = RMM_debug.matched_id;
            result.geometry_debug.armor_parity = RMM_debug.armor_parity;
            result.geometry_debug.armor_switched = RMM_debug.armor_switched;
            result.geometry_debug.direction_reversal =
                RMM_debug.direction_reversal;
            result.geometry_debug.pending_sign_conflict =
                RMM_debug.pending_sign_conflict;
            result.geometry_debug.recovered = RMM_debug.recovered;
            result.geometry_debug.temp_lost_recovery =
                RMM_debug.temp_lost_recovery;
            result.geometry_debug.candidate_is_switch =
                RMM_debug.candidate_is_switch;
            result.geometry_debug.topology_event = RMM_debug.topology_event;
            result.geometry_debug.phase_observer_valid =
                RMM_debug.phase_observer_valid;
            result.geometry_debug.phase_delta = RMM_debug.phase_delta;
            result.geometry_debug.phase_w_filtered =
                RMM_debug.phase_w_filtered;
            result.geometry_debug.phase_w_instant =
                RMM_debug.phase_w_instant;
            result.geometry_debug.best_id = RMM_debug.best_id;
            result.geometry_debug.measurement_yaw = RMM_debug.measurement_yaw;
            result.geometry_debug.predicted_yaw = RMM_debug.predicted_yaw;
            result.geometry_debug.yaw_innovation = RMM_debug.yaw_innovation;
            result.geometry_debug.measurement = RMM_debug.measurement;
            result.geometry_debug.pre_predicted = RMM_debug.pre_predicted;
            result.geometry_debug.post_predicted = RMM_debug.post_predicted;
            result.geometry_debug.pre_residual = RMM_debug.pre_residual;
            result.geometry_debug.post_residual = RMM_debug.post_residual;
            result.geometry_debug.pre_position_error =
                RMM_debug.pre_position_error;
            result.geometry_debug.post_position_error =
                RMM_debug.post_position_error;
            result.geometry_debug.residual_radial = RMM_debug.residual_radial;
            result.geometry_debug.residual_tangential =
                RMM_debug.residual_tangential;
            result.geometry_debug.nis_xyz = RMM_debug.nis_xyz;
            result.geometry_debug.nis_yaw = RMM_debug.nis_yaw;
            result.geometry_debug.yaw_variance_scale =
                RMM_debug.yaw_variance_scale;
            result.geometry_debug.p_x_m2 = RMM_debug.p_x_m2;
            result.geometry_debug.p_vx_m2_s2 = RMM_debug.p_vx_m2_s2;
            result.geometry_debug.p_y_m2 = RMM_debug.p_y_m2;
            result.geometry_debug.p_vy_m2_s2 = RMM_debug.p_vy_m2_s2;
            result.geometry_debug.hypothetical_scaled_nis =
                RMM_debug.hypothetical_scaled_nis;
            result.geometry_debug.hypothetical_scaled_nis_contribution =
                RMM_debug.hypothetical_scaled_nis_contribution;
            result.geometry_debug.geometry_valid = RMM_debug.geometry_valid;
            result.geometry_debug.geometry_update_allowed =
                RMM_debug.geometry_update_allowed;
            result.geometry_debug.geometry_preserved =
                RMM_debug.geometry_preserved;
            result.geometry_debug.updated = RMM_debug.updated;
            result.geometry_debug.measurement_valid =
                RMM_debug.measurement_valid;
            result.geometry_debug.current_armor_id =
                RMM_debug.current_armor_id;
            if (result.yaw_debug.available) {
                result.yaw_debug.ekf_yaw_rad = RMM_state.yaw;
                result.yaw_debug.ekf_w_rad_s = RMM_state.w;
                result.yaw_debug.nis = RMM_debug.nis >= 0.0
                    ? RMM_debug.nis
                    : std::numeric_limits<double>::quiet_NaN();
                result.yaw_debug.ekf_state = RMM_debug.tracker_state;
                result.yaw_debug.matched_armor_id = RMM_debug.matched_id;
                result.yaw_debug.armor_switched = RMM_debug.armor_switched;
            }
            cv::putText(RMM_visualize_frame, 
                "EKF w:"+std::to_string(RMM_state.w),
                cv::Point2f(20,80), 
                cv::FONT_HERSHEY_COMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 1, 8, false);
            cv::putText(RMM_visualize_frame, 
                cv::format("T:%.3f  dt:%.1f ms", RMM_update_time,
                           RMM_debug.dt_s * 1000.0),
                cv::Point2f(20,110), 
                cv::FONT_HERSHEY_COMPLEX, 0.7, 
                cv::Scalar(0, 255, 0), 1, 8, false);
            cv::putText(RMM_visualize_frame,
                "EKF:"+RMM_debug.tracker_state+
                " id:"+std::to_string(RMM_debug.matched_id)+
                " NIS:"+std::to_string(RMM_debug.nis),
                cv::Point2f(20,320),
                cv::FONT_HERSHEY_COMPLEX, 0.55,
                cv::Scalar(0, 255, 255), 1, 8, false);
            cv::putText(RMM_visualize_frame,
                "SP assoc: nearest-3 angle+bearing",
                cv::Point2f(20,345),
                cv::FONT_HERSHEY_COMPLEX, 0.55,
                cv::Scalar(0, 255, 255),
                1, 8, false);
            if (ekf_warmup_complete && ekf_target_predictor_->ready()) {
                EKFTargetPrediction RMM_pred_aim_data = ekf_target_predictor_ -> predict(total_delay);
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
                    EKFPredictedArmor& RMM_pred_aim_armor = RMM_pred_aim_data.armors[RMM_pred_aim_armor_i];
                    cv::Point2d yaw_vector = {std::sin(RMM_pred_aim_armor.yaw + choose_armor_yaw_bias_with_direction), -std::cos(RMM_pred_aim_armor.yaw + choose_armor_yaw_bias_with_direction)};
                    unit_center_v_dot_yaw[RMM_pred_aim_armor_i] = cam_to_center_vector.dot(yaw_vector);
                }
                std::pair<int, int> nearest_two_idx = findTwoSmallestIndices(unit_center_v_dot_yaw);
                auto nearest_armor = RMM_pred_aim_data.armors[nearest_two_idx.first];
                auto second_nearest_armor = RMM_pred_aim_data.armors[nearest_two_idx.second];
                auto chosen_armor = nearest_armor;

                if (abs(RMM_state.w) < ekf_fire_control_data.low_vyaw_threshold) {
                    cv::Point2d yaw_vector_1 = {std::sin(nearest_armor.yaw), -std::cos(nearest_armor.yaw)};
                    double yaw_bias_1 = acos(-unit_cam_to_center_vector.dot(yaw_vector_1));
                    cv::Point2d yaw_vector_2 = {std::sin(second_nearest_armor.yaw), -std::cos(second_nearest_armor.yaw)};
                    double yaw_bias_2 = acos(-unit_cam_to_center_vector.dot(yaw_vector_2));
                    if (abs(yaw_bias_1 - yaw_bias_2) < ekf_fire_control_data.low_vyaw_change_target_delta_yaw_threshold) {
                        float delta_yaw_1 = nearest_armor.yaw - ekf_fire_control_data.last_target_yaw;
                        delta_yaw_1 = atan2(sin(delta_yaw_1), cos(delta_yaw_1));
                        float delta_yaw_2 = second_nearest_armor.yaw - ekf_fire_control_data.last_target_yaw;
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
                // Keep aiming continuous, but do not fire unless the SuperPower tracker is stably TRACKING.
                if (!ekf_target_predictor_->ready() ||
                    RMM_debug.armor_switched) {
                    fire_flag = false;
                }

                ekf_has_aim = true;
                ekf_aim_world = predicted_aim_pos;
                
                cv::circle(RMM_visualize_frame, 
                    cv::Point2f(400+chosen_armor.x/RMM_visualize_zoom_out_factor, 400-chosen_armor.y/RMM_visualize_zoom_out_factor), 8, 
                    cv::Scalar(0, 0, 255), 2);
                cv::circle(RMM_visualize_frame, 
                    cv::Point2f(400+predicted_aim_pos.x/RMM_visualize_zoom_out_factor, 400-predicted_aim_pos.y/RMM_visualize_zoom_out_factor), 8, 
                    cv::Scalar(0, 255, 255), 2);
                cv::putText(RMM_visualize_frame, 
                    "r1:"+std::to_string(RMM_pred_aim_data.r1),
                    cv::Point2f(20,140), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
                cv::putText(RMM_visualize_frame, 
                    "r2:"+std::to_string(RMM_pred_aim_data.r2),
                    cv::Point2f(300,140), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
                cv::putText(RMM_visualize_frame, 
                    "flip:"+std::to_string(ekf_target_predictor_->debugFlipFlag()),
                    cv::Point2f(580,140), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
                cv::putText(RMM_visualize_frame, 
                    "center_z:"+std::to_string(RMM_pred_aim_data.center_z), 
                    cv::Point2f(20,170), 
                    cv::FONT_HERSHEY_COMPLEX, 0.7, 
                    cv::Scalar(0, 255, 0), 1, 8, false);
                cv::putText(RMM_visualize_frame, 
                    "alternate_z:"+std::to_string(RMM_pred_aim_data.alternate_z),
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
                cv::Point2f(400+RMM_pred_now_data.center_x/RMM_visualize_zoom_out_factor + std::sin(RMM_state.yaw)*1000/RMM_visualize_zoom_out_factor,
                            400-RMM_pred_now_data.center_y/RMM_visualize_zoom_out_factor + std::cos(RMM_state.yaw)*1000/RMM_visualize_zoom_out_factor),
                cv::Scalar(255, 0, 255), 2);

            // -----------------------------------------------------------------
            // Pure SuperPower-EKF visualization.
            // Semantics intentionally follow standalone v4 drawReplay():
            // measurement + EKF center + four armor hypotheses + matched-id
            // highlight + yaw arrows + NIS/residual/phase diagnostics.
            // -----------------------------------------------------------------
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

                // Match standalone replay semantics: world origin is drawn as camera/reference origin.
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
                    cv::putText(RMM_visualize_frame, "real PnP",
                                p + cv::Point(10, -10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                                cv::Scalar(180, 0, 180), 1, cv::LINE_AA);
                }

                const cv::Point center_p =
                    world_to_top(RMM_pred_now_data.center_x,
                                 RMM_pred_now_data.center_y);
                cv::circle(RMM_visualize_frame, center_p, 6,
                           cv::Scalar(0, 125, 0), -1, cv::LINE_AA);
                cv::putText(RMM_visualize_frame, "EC",
                            center_p + cv::Point(8, -8),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45,
                            cv::Scalar(0, 125, 0), 1, cv::LINE_AA);

                for (int i = 0;
                     i < static_cast<int>(RMM_pred_now_data.armors.size()); ++i) {
                    const auto& armor = RMM_pred_now_data.armors[i];
                    const cv::Point p = world_to_top(armor.x, armor.y);
                    const bool matched = (i == RMM_debug.matched_id);
                    const cv::Scalar color =
                        matched ? cv::Scalar(0, 205, 0)
                                : cv::Scalar(0, 125, 0);

                    cv::circle(RMM_visualize_frame, p,
                               matched ? 9 : 6, color,
                               matched ? -1 : 2, cv::LINE_AA);
                    cv::putText(RMM_visualize_frame,
                                "E" + std::to_string(i),
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
                    cv::circle(
                        EKF_vertical_frame,
                        world_to_vertical(ekf_measurement_world.y,
                                          ekf_measurement_world.z),
                        8, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
                }

                for (int i = 0;
                     i < static_cast<int>(RMM_pred_now_data.armors.size()); ++i) {
                    const auto& armor = RMM_pred_now_data.armors[i];
                    cv::drawMarker(
                        EKF_vertical_frame,
                        world_to_vertical(armor.y, armor.z),
                        i == RMM_debug.matched_id
                            ? cv::Scalar(0, 190, 0)
                            : cv::Scalar(0, 120, 0),
                        cv::MARKER_CROSS,
                        i == RMM_debug.matched_id ? 16 : 11,
                        2, cv::LINE_AA);
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

                // -------------------------------------------------------------
                // Real camera overlay: same SuperPower-EKF geometry as the standalone
                // replay, projected back through RestFrame + calibrated camera.
                // No legacy-RMM/fire-control debug primitives are drawn here.
                // Current E0-E3 come from predict(0); AIM is the selected future
                // EKF aim point after total-delay prediction.
                // -------------------------------------------------------------
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
                                   cv::Scalar(255, 128, 0),
                                   cv::MARKER_CROSS, 20, 2, cv::LINE_AA);
                    cv::putText(EKF_camera_overlay_frame, "EC",
                                center_px + cv::Point2f(8.0f, -8.0f),
                                cv::FONT_HERSHEY_SIMPLEX, 0.48,
                                cv::Scalar(255, 128, 0), 1, cv::LINE_AA);
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

                    const bool matched = (i == RMM_debug.matched_id);
                    const cv::Scalar armor_color =
                        matched ? cv::Scalar(0, 255, 0)
                                : cv::Scalar(0, 170, 0);

                    if (center_visible) {
                        cv::line(EKF_camera_overlay_frame, center_px, armor_px,
                                 cv::Scalar(80, 120, 80), 1, cv::LINE_AA);
                    }
                    cv::circle(EKF_camera_overlay_frame, armor_px,
                               matched ? 9 : 6, armor_color,
                               matched ? -1 : 2, cv::LINE_AA);
                    cv::putText(EKF_camera_overlay_frame,
                                "E" + std::to_string(i),
                                armor_px + cv::Point2f(8.0f, -8.0f),
                                cv::FONT_HERSHEY_SIMPLEX, 0.48,
                                armor_color, 1, cv::LINE_AA);
                }

                // Measurement is an EKF input, so show it as a small purple ring.
                if (ekf_has_measurement) {
                    cv::Point2f measurement_px;
                    if (project_world_to_camera(ekf_measurement_world,
                                                measurement_px)) {
                        cv::circle(EKF_camera_overlay_frame, measurement_px, 7,
                                   cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
                        cv::putText(EKF_camera_overlay_frame, "MEAS",
                                    measurement_px + cv::Point2f(8.0f, 16.0f),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.42,
                                    cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
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
        // ======================== SuperPower EKF ======================== END
    }

    drawYawMeasurementPanel(frame, result.yaw_debug);
    drawGeometryPanel(frame, result.geometry_debug);

    // 统一转换回pnp相机坐标系    
    predicted_aim_pos = rest_frame_ -> worldToPnpP3f(predicted_aim_pos);
    predicted_armor_pos = rest_frame_ -> worldToPnpP3f(predicted_armor_pos);

    // 弹道解算
    RCLCPP_DEBUG(node->get_logger(), "aim pos: (%.2f, %.2f, %.2f)",
                predicted_aim_pos.x, predicted_aim_pos.y, predicted_aim_pos.z);
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
    RCLCPP_DEBUG(node->get_logger(), "aim center yaw pitch: (%.2f, %.2f)",
            aim_yaw_pitch.x, aim_yaw_pitch.y);
    
    oscilloscope_common_ -> addDataPoint(((float)(result.fire_flag))/11.0, 1);
    oscilloscope_common_ -> update();
    // oscilloscope_common_ -> show();
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
