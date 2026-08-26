// ArmorTracker.h
#ifndef ARMOR_TRACKER_H
#define ARMOR_TRACKER_H

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "2d_armor_detector/Armor.h"
#include "predictor/PositionPredictor2D.h"



class ArmorTracker {
public:
    ArmorTracker(std::shared_ptr<YAML::Node> config_file_ptr);
    ~ArmorTracker();


    void preProcess(const cv::Point2f& ground_stable_point);
    void addArmor(Armor& armor, int number, bool is_large, bool not_slant, float confidence);
    std::vector<ArmorResult> afterProcess();

private:
    struct TrackedArmor {
        int track_id = -1;
        int number;
        int tracking_count;
        std::chrono::steady_clock::time_point last_seen;
        cv::Point2f center_last_seen;
        bool is_steady_tracked;
        bool is_tracked_now;
        Armor armor_last_seen;
        float confidence;
        bool is_large;
        bool not_slant;
        PositionPredictor2D predictor; 
        std::vector<cv::Point2f> predictions;
        cv::Point2f center_predicted;
        int prediction_index;
        cv::Point2f last_ground_stable_point;

        TrackedArmor(int track_id, int number, std::chrono::steady_clock::time_point seen_time, cv::Point2f center,
            Armor armor, float confidence, bool is_large, bool not_slant, int fit_step, cv::Point2f ground_stable_point) : 
        track_id(track_id), number(number), tracking_count(1), last_seen(seen_time), center_last_seen(center), is_steady_tracked(false),
        is_tracked_now(true), armor_last_seen(armor), confidence(confidence), is_large(is_large), not_slant(not_slant),
        predictor(fit_step), center_predicted(center), prediction_index(0), last_ground_stable_point(ground_stable_point) {
            predictor.addPoint(center);
        }
    };

    int next_track_id_ = 0;

    std::shared_ptr<YAML::Node> config_file_ptr;

    std::vector<TrackedArmor> tracked_armors;

    int classify_classes;

    int MAX_TRACKING_AGE_MS;
    int MIN_TRACKING_COUNT;
    float IS_NEAR_MAX_DIST_RATIO;
    int armor_tracker_fit_step;
    int armor_tracker_predict_step;

    cv::Point2f now_ground_stable_point;

    std::chrono::time_point<std::chrono::steady_clock> current_time;

    bool isNearPreviousCenter(const Armor& current_armor, 
                                           const cv::Point2f& ground_stable_point,
                                           const TrackedArmor& previous_tracked_armor, 
                                           float max_dist_ratio = -1.0);
};

#endif // ARMOR_TRACKER_H
