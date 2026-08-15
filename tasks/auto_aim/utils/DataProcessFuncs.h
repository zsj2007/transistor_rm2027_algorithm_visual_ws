#pragma once
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
