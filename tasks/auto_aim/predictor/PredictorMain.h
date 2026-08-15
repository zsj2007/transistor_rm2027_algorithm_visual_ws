#pragma once
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

#include "predictor/AllPredictor.h"
#include "2d_armor_detector/Armor.h"
#include "predictor/PredictorSwitcher.h"

class PredictorMain {
public:
    PredictorMain(std::shared_ptr<YAML::Node> config_file_ptr, 
        std::chrono::time_point<std::chrono::steady_clock> node_start_time, 
        std::shared_ptr<ArmorSolver> armor_solver_,
        std::shared_ptr<BallisticSolver> ballistic_solver_,
        std::shared_ptr<RestFrame> rest_frame_, std::shared_ptr<FrameRateCounter> fps_counter
    ) : config_file_ptr(config_file_ptr), node_start_time(node_start_time), 
    armor_solver_(armor_solver_), ballistic_solver_(ballistic_solver_),
    rest_frame_(rest_frame_), fps_counter(fps_counter) {

        classify_classes = (*config_file_ptr)["classify_classes"].as<int>();
        // all_predictors_.resize(classify_classes);
        for (size_t all_predictors_index = 0; all_predictors_index < classify_classes; all_predictors_index++) {
            all_predictors_.push_back(std::make_shared<AllPredictor>(
                config_file_ptr, node_start_time, armor_solver_,
                ballistic_solver_, rest_frame_, fps_counter, static_cast<ArmorType::ArmorType>(all_predictors_index)));
        }

        pitch_integration_max_degree = (*config_file_ptr)["pitch_integration_max_degree"].as<float>();
        pitch_integration_min_degree = (*config_file_ptr)["pitch_integration_min_degree"].as<float>();
        yaw_integration_max_degree = (*config_file_ptr)["yaw_integration_max_degree"].as<float>();
        yaw_integration_min_degree = (*config_file_ptr)["yaw_integration_min_degree"].as<float>();
        command_picth_kp = (*config_file_ptr)["command_picth_kp"].as<float>();
        command_picth_integration_speed = (*config_file_ptr)["command_picth_integration_speed"].as<float>();
        command_picth_integration_max_speed_degree = (*config_file_ptr)["command_picth_integration_max_speed_degree"].as<float>();
        command_yaw_kp = (*config_file_ptr)["command_yaw_kp"].as<float>();
        command_yaw_integration_speed = (*config_file_ptr)["command_yaw_integration_speed"].as<float>();
        command_yaw_integration_max_speed_degree = (*config_file_ptr)["command_yaw_integration_max_speed_degree"].as<float>();
    }

    PredictorResult step(std::vector<ArmorResult>& classifyResults, cv::Mat& frame, PredictorType::PredictorType predictor_type, ArmorType::ArmorType priority_armor,  bool auto_aim_switch, bool mcu_yaw_online);
    void update_serial_info(float bullet_velocity, float last_pitch_rad_delayed, float last_yaw_rad_delayed, float total_yaw_rad_delayed);

    void reset_yaw_integration();

private:
    std::shared_ptr<YAML::Node> config_file_ptr; 
    std::chrono::time_point<std::chrono::steady_clock> node_start_time;
    std::shared_ptr<ArmorSolver> armor_solver_;
    std::shared_ptr<BallisticSolver> ballistic_solver_;
    std::shared_ptr<RestFrame> rest_frame_;
    std::shared_ptr<FrameRateCounter> fps_counter;

    int classify_classes;
    std::vector<std::shared_ptr<AllPredictor>> all_predictors_;


    float last_pitch_rad_delayed_ = 0;
    float last_yaw_rad_delayed_ = 0;
    float total_yaw_rad_delayed_ = 0;

    float pitch_integration = 0.0;
    float yaw_integration = 0.0;

    float pitch_integration_max_degree;
    float pitch_integration_min_degree;
    float yaw_integration_max_degree;
    float yaw_integration_min_degree;
    float command_picth_kp;
    float command_picth_integration_speed;
    float command_picth_integration_max_speed_degree;
    float command_yaw_kp;
    float command_yaw_integration_speed;
    float command_yaw_integration_max_speed_degree;
};
