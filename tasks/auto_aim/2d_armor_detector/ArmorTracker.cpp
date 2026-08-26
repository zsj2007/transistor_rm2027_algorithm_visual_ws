// ArmorTracker.cpp
#include "2d_armor_detector/ArmorTracker.h"


ArmorTracker::ArmorTracker(std::shared_ptr<YAML::Node> config_file_ptr) 
    : config_file_ptr(config_file_ptr) {

    classify_classes = (*config_file_ptr)["classify_classes"].as<int>();

    MAX_TRACKING_AGE_MS = (*config_file_ptr)["MAX_TRACKING_AGE_MS"].as<int>();
    MIN_TRACKING_COUNT = (*config_file_ptr)["MIN_TRACKING_COUNT"].as<int>();
    IS_NEAR_MAX_DIST_RATIO = (*config_file_ptr)["IS_NEAR_MAX_DIST_RATIO"].as<float>();
    armor_tracker_fit_step = (*config_file_ptr)["armor_tracker_fit_step"].as<int>();
    armor_tracker_predict_step = (*config_file_ptr)["armor_tracker_predict_step"].as<int>();
    
}

ArmorTracker::~ArmorTracker() {}

bool ArmorTracker::isNearPreviousCenter(const Armor& current_armor, 
                                           const cv::Point2f& ground_stable_point,
                                           const TrackedArmor& previous_tracked_armor, 
                                           float max_dist_ratio) {
    cv::Point2f current_center_ground_stable = current_armor.center - ground_stable_point;
    cv::Point2f previous_center_ground_stable = previous_tracked_armor.center_last_seen - previous_tracked_armor.last_ground_stable_point;
    if (max_dist_ratio < 0)
    {
        max_dist_ratio = IS_NEAR_MAX_DIST_RATIO;
    }
    // 根据装甲板最远两角点距离确定距离基础值
    float corners_max_dist = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = i + 1; j < 4; ++j) {
            float corners_dist = cv::norm(current_armor.corners[i] - current_armor.corners[j]);
            if (corners_dist > corners_max_dist) {
                corners_max_dist = corners_dist;
            }
        }
    }
    // 根据系数参数修正
    float max_dist = corners_max_dist * max_dist_ratio;
    float dist = cv::norm(current_center_ground_stable - previous_center_ground_stable);
    return dist <= max_dist;
}

void ArmorTracker::preProcess(const cv::Point2f& ground_stable_point) {
    
    now_ground_stable_point = ground_stable_point;
    current_time = std::chrono::steady_clock::now();
    
    // 更新所有目标并清理过期的跟踪目标
    for (size_t i = 0; i < tracked_armors.size(); ++i) {
        tracked_armors[i].is_tracked_now = false;
        auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - tracked_armors[i].last_seen).count();
        if (age > MAX_TRACKING_AGE_MS) {
            tracked_armors.erase(tracked_armors.begin() + i);
            --i;
        }
    }
}

void ArmorTracker::addArmor(Armor& armor, int number, bool is_large, bool not_slant, float confidence) {

    // 检测是否正在跟踪当前装甲板
    bool is_tracked = false;
    for (int j = tracked_armors.size() - 1; j >= 0; j -= 1) {
        if (!tracked_armors[j].is_tracked_now &&
            number == tracked_armors[j].number &&
            is_large == tracked_armors[j].is_large &&
            isNearPreviousCenter(armor, now_ground_stable_point, tracked_armors[j])) {
            // 若正在跟踪则更新
            tracked_armors[j].tracking_count += 1;
            tracked_armors[j].last_seen = current_time;
            tracked_armors[j].center_last_seen = armor.center;
            tracked_armors[j].is_tracked_now = true;
            tracked_armors[j].armor_last_seen = armor;
            tracked_armors[j].confidence = confidence;
            tracked_armors[j].not_slant = not_slant;
            is_tracked = true;
            break;
        }
    }
    // 若未在跟踪则添加至跟踪列表
    if(!is_tracked) {
        tracked_armors.emplace_back(next_track_id_++, number, current_time, armor.center,
            armor, confidence, is_large, not_slant, armor_tracker_fit_step, now_ground_stable_point);
    }
}

std::vector<ArmorResult> ArmorTracker::afterProcess() {

    std::vector<ArmorResult> results;

    // 更新所有目标 part1
    for (size_t i = 0; i < tracked_armors.size(); ++i) {
        if (tracked_armors[i].last_seen != current_time && tracked_armors[i].tracking_count > 0) {
            tracked_armors[i].tracking_count -= 1;
        }

        if (tracked_armors[i].tracking_count >= MIN_TRACKING_COUNT) {
            tracked_armors[i].is_steady_tracked = true;
        } else {
            tracked_armors[i].is_steady_tracked = false;
        }

        if (tracked_armors[i].is_tracked_now) {
            for (int j = 0; j < tracked_armors[i].prediction_index-1; ++j)
            {
                tracked_armors[i].predictor.addPoint(tracked_armors[i].predictions[j] - tracked_armors[i].last_ground_stable_point);
            }
            tracked_armors[i].last_ground_stable_point = now_ground_stable_point;
            tracked_armors[i].predictor.addPoint(tracked_armors[i].center_last_seen - now_ground_stable_point);
            tracked_armors[i].predictor.fitLinear(armor_tracker_fit_step);
            tracked_armors[i].predictions = tracked_armors[i].predictor.predictLinear(armor_tracker_predict_step, now_ground_stable_point);
            tracked_armors[i].prediction_index = 0;
        } else if (tracked_armors[i].prediction_index < armor_tracker_predict_step-1) {
            tracked_armors[i].prediction_index += 1;
        }
        if (tracked_armors[i].is_steady_tracked) {
            tracked_armors[i].center_predicted = tracked_armors[i].predictions[tracked_armors[i].prediction_index];
        } else {
            tracked_armors[i].center_predicted = tracked_armors[i].center_last_seen;
        }
    }
    // 输出
    for (size_t i = 0; i < tracked_armors.size(); ++i) {
        if (tracked_armors[i].is_steady_tracked) {
            results.emplace_back(tracked_armors[i].armor_last_seen, tracked_armors[i].number,
                tracked_armors[i].track_id, tracked_armors[i].confidence,
                tracked_armors[i].is_tracked_now, tracked_armors[i].is_large, tracked_armors[i].not_slant,
                tracked_armors[i].predictions, tracked_armors[i].center_predicted, tracked_armors[i].is_steady_tracked);
        }
    }

    return results;
}
