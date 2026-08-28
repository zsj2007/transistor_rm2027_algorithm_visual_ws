#pragma once
#include <deque>
#include <limits>
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
// 计算装甲板朝向损失：正对相机为 0，侧对为 0.5，背对为 1。
// camera_to_center_direction 无需预先归一化；armor_yaw 的单位为弧度。
double normalizedArmorFacingLoss(
    const cv::Point2d& camera_to_center_direction, double armor_yaw);
// 在正对损失上为非当前物理板增加切板惩罚，并返回总损失最小的板 ID。
// previous_id 无效时不施加切板惩罚；总损失相同时优先保持当前板。
int selectArmorByFacingAndSwitchPenalty(
    const std::vector<double>& facing_losses, int previous_id,
    double switch_penalty);
