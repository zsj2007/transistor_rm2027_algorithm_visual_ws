#pragma once
#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>


struct LinearRegressionResult {
    double a;
    double b;
};

LinearRegressionResult linearRegression(const std::vector<double>& xn, const std::vector<double>& yn);
double variance(const std::vector<double>& signal);
double variance(const std::deque<double>& signal);
float variance(const std::vector<float>& signal);
float variance(const std::deque<float>& signal);
std::vector<double> linearInterpolation(const std::vector<double>& data, int refineMultiple);
double meanSquaredError(const std::vector<double>& pred_value, const std::vector<double>& true_value);
double variancePoint3f(const std::vector<cv::Point3f>& points);
double meanSquaredErrorPoint3f(const std::vector<cv::Point3f>& pred_points, const std::vector<cv::Point3f>& true_points);
std::pair<int, int> findTwoSmallestIndices(const std::vector<double>& nums);

// 装甲板沿旋转方向穿过可见半周时的离散区域；枚举值同时表示选板优先级。
enum class ArmorVisibilityRegion {
    Invisible = 0,
    Disappearing = 1,
    Appearing = 2,
    GoldenShooting = 3,
};

// 计算装甲板沿旋转方向在可见半周内的有向角：0 为刚出现，pi/2 为正对，pi 为消失。
// camera_to_center_direction 无需预先归一化；rotation_direction 只使用正负号。
double directedArmorVisibilityAngle(
    const cv::Point2d& camera_to_center_direction,
    double armor_yaw,
    int rotation_direction);

// 按 [0,45)、[45,135)、[135,180) 度划分出现、黄金射击和消失区域。
ArmorVisibilityRegion classifyArmorVisibilityRegion(double directed_angle);

// 按 Golden > Appearing > Disappearing 选择物理板；同优先级时保持 current_id。
int selectArmorByVisibilityRegion(
    const std::vector<ArmorVisibilityRegion>& regions, int current_id);
