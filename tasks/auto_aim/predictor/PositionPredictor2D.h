// PositionPredictor2D.h
#ifndef POSITION_PREDICTOR_2D_H
#define POSITION_PREDICTOR_2D_H

#include <vector>
#include <opencv2/core.hpp>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <iostream>
#include "utils/PeriodFunctions.h"


class PositionPredictor2D {
public:
    // 构造函数，指定最大历史步数
    PositionPredictor2D(int max_history = 100);
    
    // 添加新的位置点（重命名为addPoint）
    void addPoint(const cv::Point2f& point);
    
    // 线性拟合函数
    void fitLinear(int steps);
    
    // 傅里叶拟合函数
    void fitFourier(int steps, int fourier_order);
    
    // 二次曲线拟合函数（新增）
    void fitQuadratic(int steps);
    
    // 线性预测函数
    std::vector<cv::Point2f> predictLinear(int pred_length, cv::Point2f ground_stable_point) const;
    
    // 傅里叶预测函数
    std::vector<cv::Point2f> predictFourier(int pred_length, cv::Point2f ground_stable_point) const;
    
    // 二次曲线预测函数（新增）
    std::vector<cv::Point2f> predictQuadratic(int pred_length, cv::Point2f ground_stable_point) const;
    
    // 获取上一次拟合的MSE
    double getLastMSE() const;
    
    // 清除历史数据
    void clearHistory();

    // 获取添加点的计数
    int getPointCount() const;
    
    // 新增函数：获取线性拟合的速度向量
    cv::Vec2f getLinearVelocity() const;

private:
    // 一维序列线性拟合
    struct LinearFitResult {
        double a;
        double b;
        std::vector<double> residuals;
        double mse;
        int fit_point_count;   // 拟合时的点计数
        int fit_steps;         // 拟合使用的步数
    };
    
    // 一维序列二次曲线拟合（新增）
    struct QuadraticFitResult {
        double a0;  // 常数项
        double a1;  // 一次项系数（速度）
        double a2;  // 二次项系数（加速度）
        double mse;
        int fit_point_count;
        int fit_steps;
    };
    
    // 傅里叶拟合结果
    struct FourierFitResult {
        std::vector<double> beta_x;
        std::vector<double> beta_y;
        int period;
        double mse;
        int fit_point_count;   // 拟合时的点计数
        int fit_steps;         // 拟合使用的步数
    };
    
    // 一维线性拟合实现
    LinearFitResult fitLinearComponent(const std::vector<float>& y, int point_count, int steps) const;
    
    // 一维二次曲线拟合实现（新增）
    QuadraticFitResult fitQuadraticComponent(const std::vector<float>& y, int point_count, int steps) const;
    
    // 傅里叶级数拟合
    std::vector<double> fitFourierSeries(const std::vector<double>& residuals, int T, int N) const;
    
    // 计算傅里叶级数
    double fourierSeries(double t, const std::vector<double>& beta, int T, int N) const;
    
    int max_history_;          // 最大历史步数
    std::vector<cv::Point2f> history_;  // 历史位置点
    int point_count_ = 0;      // 添加点的计数
    
    // 拟合结果
    LinearFitResult linear_fit_x_;
    LinearFitResult linear_fit_y_;
    QuadraticFitResult quadratic_fit_x_;  // 新增
    QuadraticFitResult quadratic_fit_y_;  // 新增
    FourierFitResult fourier_fit_;
    
    bool linear_fitted_ = false;  // 线性拟合是否完成
    bool quadratic_fitted_ = false; // 二次曲线拟合是否完成（新增）
    bool fourier_fitted_ = false; // 傅里叶拟合是否完成

    double last_mse_ = 0.0;
};

#endif // POSITION_PREDICTOR_2D_H