#pragma once
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <memory>
#include <optional>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <limits>

#include <3d_processing/BallisticSolver.h>
#include "3d_processing/ArmorSolver.h"
#include "2d_armor_detector/Armor.h"
#include "utils/FrameRateCounter.h"
#include "3d_processing/RestFrame.h"
#include "visualizer/DataVisualizer.h"
#include "utils/SimpleDataFilter.h"
#include "EKF/SuperPowerPredictor.h"
#include "predictor/PredictorSwitcher.h"  // 保留 PredictorType 接口兼容

struct YawMeasurementDebug {
    bool available = false;
    double yaw_raw_rad = std::numeric_limits<double>::quiet_NaN();
    double yaw_refined_rad = std::numeric_limits<double>::quiet_NaN();
    double yaw_used_rad = std::numeric_limits<double>::quiet_NaN();
    double yaw_delta_rad = std::numeric_limits<double>::quiet_NaN();
    double reprojection_rmse_raw_px =
        std::numeric_limits<double>::quiet_NaN();
    double reprojection_rmse_refined_px =
        std::numeric_limits<double>::quiet_NaN();
    double facing_angle_rad = std::numeric_limits<double>::quiet_NaN();
    bool refined_valid = false;
    std::string refinement_status = "DISABLED";
};

struct GeometryDebug {
    bool available = false;
    std::string ekf_state = "LOST";
    double r1_m = std::numeric_limits<double>::quiet_NaN();
    double r2_m = std::numeric_limits<double>::quiet_NaN();
    double h_m = std::numeric_limits<double>::quiet_NaN();
    double p_r1_m2 = std::numeric_limits<double>::quiet_NaN();
    double p_r2_m2 = std::numeric_limits<double>::quiet_NaN();
    double p_h_m2 = std::numeric_limits<double>::quiet_NaN();
    double w_rad_s = std::numeric_limits<double>::quiet_NaN();
    double nis = std::numeric_limits<double>::quiet_NaN();
    int matched_armor_id = -1;
    int armor_parity = -1;
    bool armor_switched = false;
    int joint_second_id = -1;
    double joint_nis = std::numeric_limits<double>::quiet_NaN();
    std::string joint_status = "SINGLE";

    // 同帧单板 EKF 仅用于对比可视化，不参与火控。
    bool comparison_available = false;
    double joint_minus_baseline_ground_range_m =
        std::numeric_limits<double>::quiet_NaN();
    double center_separation_m =
        std::numeric_limits<double>::quiet_NaN();
    bool geometry_valid = false;
    bool geometry_update_allowed = false;
    bool geometry_preserved = false;
};

struct JointEkfTrackPair {
    int number = -1;
    int track_id_a = -1;
    int track_id_b = -1;
};

struct PredictorResult {
    bool reset = true;
    float command_pitch = 0.0;
    float command_yaw = 0.0;
    float command_delta_pitch = 0.0;
    float command_delta_yaw = 0.0;
    bool fire_flag = false;
    PredictorType::PredictorType predictor_type = PredictorType::None;
    ArmorType::ArmorType armor_type = ArmorType::Hero;
    float pixel_horizontal_center_distance = 1e10;
    float latest_armor_distance = 1e10;
    bool integrating = false;
    YawMeasurementDebug yaw_debug;
    GeometryDebug geometry_debug;

    struct {
        cv::Mat RMM_visualize_frame;      // EKF 顶视图
        cv::Mat EKF_vertical_frame;        // EKF 纵向视图
        cv::Mat EKF_camera_overlay_frame;  // 相机画面与 EKF 投影
        cv::Mat common_debug_oscilloscope_frame;
    } info_images;
};

struct EKF_fire_result_t {
    bool aim_center;
    bool fire;
};

class AllPredictor {
public:
    AllPredictor(std::shared_ptr<YAML::Node> config_file_ptr, 
        std::chrono::time_point<std::chrono::steady_clock> node_start_time, 
        std::shared_ptr<ArmorSolver> armor_solver_,
        std::shared_ptr<BallisticSolver> ballistic_solver_,
        std::shared_ptr<RestFrame> rest_frame_, std::shared_ptr<FrameRateCounter> fps_counter,
        ArmorType::ArmorType armor_class
    ) : config_file_ptr(config_file_ptr), node_start_time(node_start_time), 
    armor_solver_(armor_solver_), ballistic_solver_(ballistic_solver_),
    rest_frame_(rest_frame_), fps_counter(fps_counter), armor_class(armor_class) {
        // 初始化参数
        const float fix_bullet_velocity = (*config_file_ptr)["FIX_BULLET_VELOCITY"]
            ? (*config_file_ptr)["FIX_BULLET_VELOCITY"].as<float>() : -1.0f;
        if (fix_bullet_velocity >= 0.0f) {
            bullet_velocity_ = fix_bullet_velocity;
        } else {
            bullet_velocity_ = (*config_file_ptr)["bullet_velocity_"].as<float>();
        }
        
        const YAML::Node& camera_matrix_Node = (*config_file_ptr)["camera_matrix"];
        yaw_rad_to_x_pixel_ratio = camera_matrix_Node[0][0].as<float>(); 
        pitch_rad_to_y_pixel_ratio = camera_matrix_Node[1][1].as<float>(); 

        const bool show_windows =
            (*config_file_ptr)["SHOW_WINDOWS"]
                ? (*config_file_ptr)["SHOW_WINDOWS"].as<bool>()
                : false;
        oscilloscope_common_ = std::make_shared<Oscilloscope>(
            640, 480,
            "Common Debug Oscilloscope " + std::to_string(armor_class),
            2,
            cv::Scalar(0, 0, 0),
            cv::Scalar(0, 255, 0),
            show_windows);
        oscilloscope_common_ -> setScale(2.0);
        oscilloscope_common_ -> setOffset(-1.0);

        ekf_fire_control_data.after_target_change_ceasefire_ms = (*config_file_ptr)["after_target_change_ceasefire_ms"].as<int>();
        ekf_fire_control_data.before_target_change_ceasefire_ms = (*config_file_ptr)["before_target_change_ceasefire_ms"].as<int>();
        ekf_fire_control_data.aim_center_vyaw_lower_threshold = (*config_file_ptr)["aim_center_vyaw_lower_threshold"].as<float>();
        ekf_fire_control_data.aim_center_vyaw_upper_threshold = (*config_file_ptr)["aim_center_vyaw_upper_threshold"].as<float>();
        ekf_fire_control_data.aim_center_yaw_bias_expand = (*config_file_ptr)["aim_center_yaw_bias_expand"].as<float>();
        ekf_fire_control_data.low_vyaw_change_target_delta_yaw_threshold = M_PI / 180.0 * (*config_file_ptr)["low_vyaw_change_target_delta_yaw_threshold_degree"].as<float>();
        ekf_fire_control_data.low_vyaw_threshold = (*config_file_ptr)["low_vyaw_threshold"].as<float>();

        pre_predict_time = (*config_file_ptr)["pre_predict_time"].as<float>();
        pre_predict_time_not_aim = (*config_file_ptr)["pre_predict_time_not_aim"].as<float>();

        extra_predict_time = (*config_file_ptr)["extra_predict_time"].as<float>();
        choose_armor_yaw_bias = M_PI / 180.0 * (*config_file_ptr)["choose_armor_yaw_bias_degree"].as<float>();
        choose_armor_region_hysteresis = M_PI / 180.0F *
            ((*config_file_ptr)["choose_armor_region_hysteresis_degree"]
                 ? std::max(0.0F, (*config_file_ptr)["choose_armor_region_hysteresis_degree"].as<float>())
                 : 5.0F);
        choose_armor_switch_confirm_frames =
            (*config_file_ptr)["choose_armor_switch_confirm_frames"]
                ? std::max(1, (*config_file_ptr)["choose_armor_switch_confirm_frames"].as<int>())
                : 2;
        const YAML::Node comparison =
            (*config_file_ptr)["superpower_ekf"]["comparison"];
        comparison_enabled_ =
            comparison && comparison["enabled"] &&
            comparison["enabled"].as<bool>();
    }

    PredictorResult step(
        std::vector<ArmorResult>& classifyResults,
        const std::vector<JointEkfTrackPair>& joint_pairs,
        cv::Mat& frame,
        double frame_timestamp_s);
    void update_serial_info(float bullet_velocity, float last_pitch_rad_delayed, float last_yaw_rad_delayed, float total_yaw_rad_delayed);
    void resetTarget();
    void startTarget();
    bool targetActive() const;
    std::optional<std::string> ekfTrackerState() const;
    ArmorResult* selectCurrentMeasurement(std::vector<ArmorResult>& candidates);
private:
    // 低速时锁存旋转方向；反向连续确认后才改变 Appearing/Disappearing 定义。
    void updateLatchedRotationDirection(double angular_velocity);
    // 用预测到弹丸到达时刻的四块装甲区域，经过迟滞和连续确认后选择瞄准板。
    int selectPredictedArmorByRegion(
        const EKFTargetPrediction& prediction,
        const cv::Point2d& camera_to_center_direction);

    ArmorType::ArmorType armor_class;
    bool armor_is_large;

    std::shared_ptr<YAML::Node> config_file_ptr; 
    std::chrono::time_point<std::chrono::steady_clock> node_start_time;
    std::shared_ptr<ArmorSolver> armor_solver_;
    std::shared_ptr<BallisticSolver> ballistic_solver_;
    std::shared_ptr<RestFrame> rest_frame_;
    std::shared_ptr<FrameRateCounter> fps_counter;

    float last_total_delay_ = 0.0;
    bool target_active_ = false;
    std::chrono::steady_clock::time_point latest_predicting_start_time;

    std::shared_ptr<Oscilloscope> oscilloscope_common_;

    std::shared_ptr<SuperPowerPredictor> ekf_target_predictor_;
    std::shared_ptr<SuperPowerPredictor> baseline_ekf_target_predictor_;
    bool comparison_enabled_ = false;

    float bullet_velocity_;
    float last_pitch_rad_delayed_ = 0;
    float last_yaw_rad_delayed_ = 0;
    float total_yaw_rad_delayed_ = 0;

    float yaw_rad_to_x_pixel_ratio;
    float pitch_rad_to_y_pixel_ratio;
    cv::Point2f last_aim_yaw_pitch_;

    cv::Point3f last_rest_frame_pos = {0.0, 0.0, 0.0};

    float last_pixel_horizontal_center_distance = 1e10;

    bool has_valid_ballistic = false;
    
    float init_r = 200.0;  // 普通四装甲初始半径，单位 mm

    struct EKF_fire_control_data_t {
        int after_target_change_ceasefire_ms;
        int before_target_change_ceasefire_ms;
        float aim_center_vyaw_lower_threshold;
        float aim_center_vyaw_upper_threshold;
        float aim_center_yaw_bias_expand;
        float low_vyaw_change_target_delta_yaw_threshold;
        float low_vyaw_threshold;

        bool aim_center_schmitt_trigger = false;
        bool new_target = true;
        float last_target_yaw;
        std::chrono::steady_clock::time_point last_target_yaw_jump_time;
    } ekf_fire_control_data;

    EKF_fire_result_t EKF_fire_control(EKFPredictedArmor chosen_armor, EKFTargetState ekf_state, float yaw_bias, bool is_large_armor, cv::Point2d cam_to_center_vector, float choose_armor_yaw_bias_with_direction);

    float latest_armor_distance = 1e10;

    float pre_predict_time;
    float pre_predict_time_not_aim;

    float choose_armor_yaw_bias;

    // 区域切换迟滞与状态：候选板连续确认后才提交，低速时保持上次可靠方向。
    float choose_armor_region_hysteresis = 5.0F * M_PI / 180.0F;
    int choose_armor_switch_confirm_frames = 2;
    int last_selected_aim_id_ = -1;
    int pending_selected_aim_id_ = -1;
    int pending_selected_aim_frames_ = 0;
    int latched_rotation_direction_ = 1;
    int pending_rotation_direction_ = 0;
    int pending_rotation_direction_frames_ = 0;

    float extra_predict_time;

};
