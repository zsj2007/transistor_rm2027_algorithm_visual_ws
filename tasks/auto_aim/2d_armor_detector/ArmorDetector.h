// ArmorDetector.h
#ifndef ARMOR_DETECTOR_H
#define ARMOR_DETECTOR_H

#include <opencv2/opencv.hpp>
#include "LightBar.h"
#include "2d_armor_detector/Armor.h"
#include <vector>
#include <yaml-cpp/yaml.h>
#define _USE_MATH_DEFINES // 启用数学常量
#include <cmath>
#include <algorithm>
#include <thread>

class ArmorDetector {
public:
    ArmorDetector(std::shared_ptr<YAML::Node> config_file_ptr)
    : config_file_ptr(config_file_ptr) {
        max_angle_diff = (*config_file_ptr)["max_angle_diff"].as<float>();
        max_height_diff_ratio = (*config_file_ptr)["max_height_diff_ratio"].as<float>();
        min_light_distance = (*config_file_ptr)["min_light_distance"].as<float>();
        max_light_distance = (*config_file_ptr)["max_light_distance"].as<float>();
        min_armor_confidence = (*config_file_ptr)["min_armor_confidence"].as<float>();
        max_expected_small_distance_mismatch_ratio = (*config_file_ptr)["max_expected_small_distance_mismatch_ratio"].as<float>();
        max_expected_large_distance_mismatch_ratio = (*config_file_ptr)["max_expected_large_distance_mismatch_ratio"].as<float>();
        max_length_direction_mismatch_ratio = (*config_file_ptr)["max_length_direction_mismatch_ratio"].as<float>();
    }
    std::vector<Armor> detectArmors(const std::vector<Light>& lights);
private:
    float max_angle_diff;
    float max_height_diff_ratio;
    float min_light_distance;
    float max_light_distance;
    float min_armor_confidence;
    float max_expected_small_distance_mismatch_ratio;
    float max_expected_large_distance_mismatch_ratio;
    float max_length_direction_mismatch_ratio;
    
    bool isArmorPair(const Light& l1, const Light& l2);
    float getArmorConfidence(const Light& l1, const Light& l2);
    
    std::shared_ptr<YAML::Node> config_file_ptr;
};

#endif // ARMOR_DETECTOR_H
