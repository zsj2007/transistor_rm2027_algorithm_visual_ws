// PeriodicDataPredictor.h
#ifndef PERIODIC_DATA_PREDICTOR_H
#define PERIODIC_DATA_PREDICTOR_H

#include <vector>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <iostream>
#include "utils/PeriodFunctions.h"
#include <algorithm>

class PeriodicDataPredictor {
public:
    // 构造函数，指定最大历史步数和傅里叶阶数（默认1阶，保持兼容）
    PeriodicDataPredictor(int max_history = 100, int fourier_order = 1);
    
    // 添加新的数据点
    void addPoint(double point);
    
    // 设置周期
    void setPeriod(int period);
    
    // 获取周期
    int getPeriod() const;
    
    // 设置傅里叶级数的阶数
    void setFourierOrder(int order);
    
    // 获取傅里叶级数的阶数
    int getFourierOrder() const;
    
    // 获取指定时间的平滑后数据
    double smooth(int time_index) const;
    
    // 判断指定点平滑后数据是否位于上升沿（导数>0）
    bool isRising(int time_index, double compare_threshold = 0.0) const;

    // 计算相位是否为正半周期
    bool isUpper(int time_index, double compare_threshold = 0.0) const;

    // 获取a0
    double getA0() const;
    
    // 获取指定阶数的傅里叶系数a
    double getCoefficientA(int order) const;
    
    // 获取指定阶数的傅里叶系数b
    double getCoefficientB(int order) const;
    
    // 清除历史数据
    void clearHistory();

    // 获取添加点的计数
    int getPointCount() const;
    
    void autoFindPeriod();
    
    double getFitMse();
    
private:
    // 计算傅里叶系数
    void computeFourierCoefficients() const;
    
    // 计算傅里叶级数的导数
    double computeDerivative(int time_index) const;
    
    int max_history_;          // 最大历史步数
    std::vector<double> history_;  // 历史数据点
    int point_count_ = 0;      // 添加点的计数
    int period_ = 0;           // 周期
    int fourier_order_ = 1;    // 傅里叶级数阶数
    
    // 傅里叶系数（支持多阶）
    mutable double a0_ = 0.0;
    mutable std::vector<double> a_coeffs_;  // a1, a2, ..., an
    mutable std::vector<double> b_coeffs_;  // b1, b2, ..., bn
    
    mutable bool coefficients_dirty_ = true; // 标记系数是否需要重新计算
};

#endif // PERIODIC_DATA_PREDICTOR_H
