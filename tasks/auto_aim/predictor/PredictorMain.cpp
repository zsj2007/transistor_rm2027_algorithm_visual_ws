#include "predictor/PredictorMain.h"

#include <filesystem>
#include <iomanip>
#include <iostream>


namespace {
constexpr const char* kSuperPowerCsvHeader =
    "frame_index,"
    "source_t_s,"
    "process_current_frame,"
    "target_state,"
    "target_type,"
    "detect_count,"
    "temp_lost_count,"
    "geometry_available,"
    "ekf_dt_s,"
    "ekf_state,"
    "tracker_state_before,"
    "center_x_m,"
    "center_y_m,"
    "center_z_m,"
    "vx_m_s,"
    "vy_m_s,"
    "vz_m_s,"
    "state_yaw_rad,"
    "w_rad_s,"
    "r1_m,"
    "r2_m,"
    "h_m,"
    "P_r1_m2,"
    "P_r2_m2,"
    "P_h_m2,"
    "matched_id,"
    "parity,"
    "armor_switched,"
    "NIS,"
    "has_measurement,"
    "measurement_number,"
    "measurement_x_m,"
    "measurement_y_m,"
    "measurement_z_m,"
    "measurement_yaw_rad,"
    "input_world_x_m,"
    "input_world_y_m,"
    "input_world_z_m,"
    "input_yaw_raw_rad,"
    "input_yaw_refined_rad,"
    "input_yaw_used_rad,"
    "pre_position_error_m,"
    "post_position_error_m,"
    "updated,"
    "measurement_valid,"
    "fire_flag,"
    "integrating,"
    "command_pitch_rad,"
    "command_yaw_rad";
}

void PredictorMain::initSuperPowerCsv()
{
    const YAML::Node root = *config_file_ptr;
    const YAML::Node diag = root["superpower_csv"];
    if (!diag || !diag["enabled"] || !diag["enabled"].as<bool>()) {
        return;
    }

    superpower_csv_enabled_ = true;
    superpower_csv_path_ =
        diag["path"] ? diag["path"].as<std::string>()
                     : std::string("logs/superpower_ekf_rm2027.csv");

    if (diag["flush_every_n"]) {
        const int n = diag["flush_every_n"].as<int>();
        if (n > 0) {
            superpower_csv_flush_every_n_ = static_cast<std::size_t>(n);
        }
    }

    const std::filesystem::path csv_path(superpower_csv_path_);
    if (csv_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(csv_path.parent_path(), ec);
        if (ec) {
            std::cerr << "[SuperPowerCSV] cannot create directory '"
                      << csv_path.parent_path().string()
                      << "': " << ec.message() << std::endl;
            superpower_csv_enabled_ = false;
            return;
        }
    }

    superpower_csv_stream_.open(
        superpower_csv_path_,
        std::ios::out | std::ios::trunc);

    if (!superpower_csv_stream_.is_open()) {
        std::cerr << "[SuperPowerCSV] cannot open "
                  << superpower_csv_path_ << std::endl;
        superpower_csv_enabled_ = false;
        return;
    }

    superpower_csv_stream_ << kSuperPowerCsvHeader << '\n';
    superpower_csv_stream_.flush();

    std::cout << "[SuperPowerCSV] logging to "
              << superpower_csv_path_
              << " | flush_every_n=" << superpower_csv_flush_every_n_
              << std::endl;
}

void PredictorMain::writeSuperPowerCsv(
    double source_timestamp_s,
    bool process_current_frame,
    const TargetManagerStatus& target_status,
    const PredictorResult& result)
{
    if (!superpower_csv_enabled_ || !superpower_csv_stream_.is_open()) {
        return;
    }

    const GeometryDebug& g = result.geometry_debug;
    const YawMeasurementDebug& y = result.yaw_debug;

    const int target_type =
        target_status.target_type.has_value()
            ? static_cast<int>(*target_status.target_type)
            : -1;

    superpower_csv_stream_ << std::setprecision(10)
        << superpower_csv_frame_index_++ << ','
        << source_timestamp_s << ','
        << (process_current_frame ? 1 : 0) << ','
        << TargetManager::stateName(target_status.state) << ','
        << target_type << ','
        << target_status.detect_count << ','
        << target_status.temp_lost_count << ','
        << (g.available ? 1 : 0) << ','
        << g.dt_s << ','
        << (g.available ? g.ekf_state : std::string("NA")) << ','
        << (g.available ? g.tracker_state_before : std::string("NA")) << ','
        << g.center_x_m << ','
        << g.center_y_m << ','
        << g.center_z_m << ','
        << g.vx_m_s << ','
        << g.vy_m_s << ','
        << g.vz_m_s << ','
        << g.state_yaw_rad << ','
        << g.w_rad_s << ','
        << g.r1_m << ','
        << g.r2_m << ','
        << g.h_m << ','
        << g.p_r1_m2 << ','
        << g.p_r2_m2 << ','
        << g.p_h_m2 << ','
        << g.matched_armor_id << ','
        << g.armor_parity << ','
        << (g.armor_switched ? 1 : 0) << ','
        << g.nis << ','
        << (result.has_measurement ? 1 : 0) << ','
        << result.measurement_number << ','
        << g.measurement[0] << ','
        << g.measurement[1] << ','
        << g.measurement[2] << ','
        << g.measurement_yaw << ','
        << (static_cast<double>(y.measurement_world_mm.x) / 1000.0) << ','
        << (static_cast<double>(y.measurement_world_mm.y) / 1000.0) << ','
        << (static_cast<double>(y.measurement_world_mm.z) / 1000.0) << ','
        << y.yaw_raw_rad << ','
        << y.yaw_refined_rad << ','
        << y.yaw_used_rad << ','
        << g.pre_position_error << ','
        << g.post_position_error << ','
        << (g.updated ? 1 : 0) << ','
        << (g.measurement_valid ? 1 : 0) << ','
        << (result.fire_flag ? 1 : 0) << ','
        << (result.integrating ? 1 : 0) << ','
        << result.command_pitch << ','
        << result.command_yaw
        << '\n';

    ++superpower_csv_row_count_;
    if (superpower_csv_row_count_ % superpower_csv_flush_every_n_ == 0) {
        superpower_csv_stream_.flush();
    }
}

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

PredictorResult PredictorMain::step(std::vector<ArmorResult>& classifyResults,
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
            // This is the only predictor advanced this frame. Other detected
            // vehicles remain available for drawing but cannot update or
            // compete with the persistent target.
            if (!all_predictors_[active_index]->targetActive()) {
                all_predictors_[active_index]->resetTarget();
                all_predictors_[active_index]->startTarget();
            }
            chosen_result = all_predictors_[active_index]->step(
                target_update.target_candidates, frame, frame_timestamp_s);
        }
    }

    // Target lifecycle is the final fire-control authority. DETECTING may
    // initialize and update the predictor, and TEMP_LOST may execute pure
    // prediction, but only a valid, continuous TRACKING frame may fire or
    // accumulate PI.
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
    if (chosen_result.yaw_debug.available) {
        chosen_result.yaw_debug.target_state =
            TargetManager::stateName(target_status.state);
    }
    if (chosen_result.geometry_debug.available) {
        chosen_result.geometry_debug.target_state =
            TargetManager::stateName(target_status.state);
    }

    writeSuperPowerCsv(
        frame_timestamp_s,
        target_update.process_current_frame,
        target_status,
        chosen_result);

    return chosen_result;
}

void PredictorMain::reset_yaw_integration() {
    yaw_integration = 0.0;
}

const TargetManagerStatus& PredictorMain::targetManagerStatus() const
{
    return target_manager_->status();
}
