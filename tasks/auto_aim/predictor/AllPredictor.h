#pragma once
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#include <3d_processing/BallisticSolver.h>
#include "3d_processing/ArmorSolver.h"
#include "2d_armor_detector/Armor.h"
#include "utils/FrameRateCounter.h"
#include "3d_processing/RestFrame.h"
#include "visualizer/DataVisualizer.h"
#include "utils/SimpleDataFilter.h"
#include "predictor/RotationMotionModel.h"
#include "predictor/PredictorSwitcher.h"

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

    struct {
        cv::Mat RMM_visualize_frame;
        cv::Mat common_debug_oscilloscope_frame;
    } info_images;
};

struct RMM_fire_result_t {
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
        
        // yaw_rad_to_x_pixel_ratio = (*config_file_ptr)["yaw_rad_to_x_pixel_ratio"].as<float>(); 
        // pitch_rad_to_y_pixel_ratio = (*config_file_ptr)["pitch_rad_to_y_pixel_ratio"].as<float>(); 
        const YAML::Node& camera_matrix_Node = (*config_file_ptr)["camera_matrix"];
        yaw_rad_to_x_pixel_ratio = camera_matrix_Node[0][0].as<float>(); 
        pitch_rad_to_y_pixel_ratio = camera_matrix_Node[1][1].as<float>(); 

        reset_predictor_time = (*config_file_ptr)["reset_predictor_time"].as<float>(); 

        last_com_time = std::chrono::steady_clock::now();

        const bool show_windows =
            (*config_file_ptr)["SHOW_WINDOWS"] ? (*config_file_ptr)["SHOW_WINDOWS"].as<bool>() : false;
        oscilloscope_common_ = std::make_shared<Oscilloscope>(
            640,
            480,
            "Common Debug Oscilloscope " + std::to_string(armor_class),
            2,
            cv::Scalar(0, 0, 0),
            cv::Scalar(0, 255, 0),
            show_windows);
        oscilloscope_common_ -> setScale(2.0);
        oscilloscope_common_ -> setOffset(-1.0);

        predictor_switcher_ = std::make_shared<PredictorSwitcher>(config_file_ptr);

        RMM_fire_control_data.after_target_change_ceasefire_ms = (*config_file_ptr)["after_target_change_ceasefire_ms"].as<int>();
        RMM_fire_control_data.before_target_change_ceasefire_ms = (*config_file_ptr)["before_target_change_ceasefire_ms"].as<int>();
        RMM_fire_control_data.aim_center_vyaw_lower_threshold = (*config_file_ptr)["aim_center_vyaw_lower_threshold"].as<float>();
        RMM_fire_control_data.aim_center_vyaw_upper_threshold = (*config_file_ptr)["aim_center_vyaw_upper_threshold"].as<float>();
        RMM_fire_control_data.aim_center_yaw_bias_expand = (*config_file_ptr)["aim_center_yaw_bias_expand"].as<float>();
        RMM_fire_control_data.low_vyaw_change_target_delta_yaw_threshold = M_PI / 180.0 * (*config_file_ptr)["low_vyaw_change_target_delta_yaw_threshold_degree"].as<float>();
        RMM_fire_control_data.low_vyaw_threshold = (*config_file_ptr)["low_vyaw_threshold"].as<float>();

        pre_predict_time = (*config_file_ptr)["pre_predict_time"].as<float>();
        pre_predict_time_not_aim = (*config_file_ptr)["pre_predict_time_not_aim"].as<float>();

        extra_predict_time = (*config_file_ptr)["extra_predict_time"].as<float>();
        choose_armor_yaw_bias = M_PI / 180.0 * (*config_file_ptr)["choose_armor_yaw_bias_degree"].as<float>();
        RMM_visualize_zoom_out_factor = (*config_file_ptr)["RMM_visualize_zoom_out_factor"].as<float>();
    }

    PredictorResult step(std::vector<ArmorResult>& classifyResults, cv::Mat& frame, PredictorType::PredictorType control_predictor_type);
    void update_serial_info(float bullet_velocity, float last_pitch_rad_delayed, float last_yaw_rad_delayed, float total_yaw_rad_delayed);

    bool is_reset = false;

    std::chrono::steady_clock::time_point latest_predicting_start_time;
private:
    PredictorType::PredictorType using_predictor_type = PredictorType::None;
    ArmorType::ArmorType armor_class;
    bool armor_is_large;

    std::shared_ptr<YAML::Node> config_file_ptr; 
    std::chrono::time_point<std::chrono::steady_clock> node_start_time;
    std::shared_ptr<ArmorSolver> armor_solver_;
    std::shared_ptr<BallisticSolver> ballistic_solver_;
    std::shared_ptr<RestFrame> rest_frame_;
    std::shared_ptr<FrameRateCounter> fps_counter;

    float last_total_delay_ = 0.0;

    std::shared_ptr<Oscilloscope> oscilloscope_common_;

    std::shared_ptr<RotationMotionModel> rotation_motion_model_;

    float bullet_velocity_;
    float last_pitch_rad_delayed_ = 0;
    float last_yaw_rad_delayed_ = 0;
    float total_yaw_rad_delayed_ = 0;

    float yaw_rad_to_x_pixel_ratio;
    float pitch_rad_to_y_pixel_ratio;
    float reset_predictor_time;
    std::chrono::steady_clock::time_point last_com_time;
    cv::Point2f last_aim_yaw_pitch_;

    std::shared_ptr<PredictorSwitcher> predictor_switcher_;

    cv::Point3f last_rest_frame_pos = {0.0, 0.0, 0.0};

    float last_pixel_horizontal_center_distance = 1e10;

    bool has_valid_ballistic = false;
    
    float init_r = 250.0;

    struct RMM_fire_control_data_t {
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
    } RMM_fire_control_data;

    RMM_fire_result_t RMM_fire_control(SimpleArmor chosen_armor, RotationMotionState RMM_state, float yaw_bias, bool is_large_armor, cv::Point2d cam_to_center_vector, float choose_armor_yaw_bias_with_direction);

    float latest_armor_distance = 1e10;

    float pre_predict_time;
    float pre_predict_time_not_aim;

    float choose_armor_yaw_bias;

    float extra_predict_time;

    float RMM_visualize_zoom_out_factor;
};
