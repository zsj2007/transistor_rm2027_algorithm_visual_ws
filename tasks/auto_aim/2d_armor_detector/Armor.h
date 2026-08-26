// Armor.h
#ifndef ARMOR_H
#define ARMOR_H

#define _USE_MATH_DEFINES // 启用数学常量
#include <array>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

namespace ArmorType {
    enum ArmorType {
        Hero = 0,
        Engineer,
        Infantry1,
        Infantry2,
        Infantry3, // 5号
        Sentry,
        Outpost,
        Base,
        Middle,
        Nearest
    };
    extern std::vector<std::string> ArmorTypeStrings;
}

// 物理尺寸常量
namespace ArmorConstants {
    constexpr float LIGHT_HEIGHT = 55.0f;       // 灯条高度
    // 小装甲板
    constexpr float SMALL_ARMOR_HEIGHT = 125.0f;  
    constexpr float SMALL_ARMOR_WIDTH = 135.0f;
    // 大装甲板
    constexpr float LARGE_ARMOR_HEIGHT = 127.0f;
    constexpr float LARGE_ARMOR_WIDTH = 230.0f;
    // 高度比例约为2.27和2.31
    constexpr float SMALL_HEIGHT_RATIO = SMALL_ARMOR_HEIGHT / LIGHT_HEIGHT;
    constexpr float LARGE_HEIGHT_RATIO = LARGE_ARMOR_HEIGHT / LIGHT_HEIGHT;
    // 灯条距离和高度的比值
    constexpr float LARGE_DISTANCE_RATIO = LARGE_ARMOR_WIDTH / LIGHT_HEIGHT;
    constexpr float SMALL_DISTANCE_RATIO = SMALL_ARMOR_WIDTH / LIGHT_HEIGHT;

    // 新增用于pnp的灯条顶点参数
    constexpr float PNP_LIGHT_HEIGHT = 43.0f;       // pnp用灯条高度 // 待修正
    constexpr float SMALL_ARMOR_LIGHT_DISTANCE = 130.0f; // 待修正
    constexpr float LARGE_ARMOR_LIGHT_DISTANCE = 225.0f; // 待修正
}

struct AimResult {
    cv::Point3f position;  // 装甲板中心在枪口坐标系下的位置
    double distance;       // 距离
    bool valid;           // 解算是否有效

    double yaw;           // 装甲板Yaw角
    cv::Mat rvec;        // 装甲板旋转向量

    std::vector<double> normal_euler_angles; // RestFrame中定义的坐标系下的欧拉角

    std::vector<double> ba_global_ypr;
};

struct Armor {
    cv::RotatedRect leftLight;    // 左灯条
    cv::RotatedRect rightLight;   // 右灯条
    cv::Rect roi;                 // ROI区域
    float confidence;             // 置信度
    std::vector<cv::Point2f> corners;  // 四个角点坐标（默认小装甲板）
    cv::Point2f center;                 // 角点对角线连线焦点确定中心
    std::vector<cv::Point2f> corners_large;  // 大装甲板角点坐标（识别装甲板类型后使用）
    std::vector<cv::Point2f> corners_expanded;  // 扩大后的四个角点坐标，用于展平后识别
    std::vector<cv::Point2f> corners_narrowed;  // 缩小后的四个角点坐标，用于yolo灯条点亮识别
    float corners_expand_ratio;  // 角点坐标扩大比例
    float corners_narrow_ratio;  // 角点坐标缩小比例
    float height_correct_ratio_small;     // 角点坐标修正比例系数
    float width_correct_ratio_small;
    float height_correct_ratio_large;
    float width_correct_ratio_large;
    float is_ture_yolo_armor_brightness_ratio;

    struct {
        bool is_delayed_result = false;
        int history_frame_identifier;
    } delayed_result;

    // 默认构造函数
    Armor() : confidence(0.0f) {}
    
    // 带参数的构造函数
    Armor(const cv::RotatedRect& left, const cv::RotatedRect& right, std::shared_ptr<YAML::Node> config_file_ptr, int history_frame_identifier = -1);

    // ROI计算函数
    void calculateROI();
    
    void calculateCorners();

    cv::Point2f vecToPoint(const cv::Vec2f& vec);

    cv::Vec2f pointToVec(const cv::Point2f& point);

    cv::Point2f computeIntersection(const std::vector<cv::Point2f>& corners);

    std::vector<cv::Point2f> light_bar_corners;
    // 保存 YOLO 输出的左上、左下、右下、右上四个灯条端点。
    std::array<cv::Point2f, 4> raw_light_bar_endpoints{};

    bool is_true_yolo_armor(cv::Mat& frame);
};

struct ArmorResult {
    Armor armor;              
    int number;
    int track_id;
    float confidence;
    std::vector<cv::Point2f> corners;  
    cv::Point2f center;
    bool is_tracked_now;
    bool is_large;
    bool not_slant;
    std::vector<cv::Point2f> predictions;
    cv::Point2f center_predicted;
    bool is_steady_tracked;

    AimResult solve_armor_result;

    ArmorResult(const Armor& a, int n, int tid, float conf,
        bool is_tracked_now, bool is_large, bool not_slant,
        std::vector<cv::Point2f> predictions, cv::Point2f center_predicted, 
        bool is_steady_tracked) 
        : armor(a), number(n), track_id(tid), confidence(conf), corners(a.corners), center(a.center),
        is_tracked_now(is_tracked_now), is_large(is_large), not_slant(not_slant),
        predictions(predictions), center_predicted(center_predicted),
        is_steady_tracked(is_steady_tracked) {}
};

#endif // ARMOR_H
