// SimpleDataFilter.h
#ifndef SIMPLE_DATA_FILTER_H
#define SIMPLE_DATA_FILTER_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <deque>
#include <cmath>

class SimpleDataFilter {
public:
    // 构造函数，指定最大历史点数
    explicit SimpleDataFilter(int max_history = 100);
    
    // 添加新的数据点
    void addPoint(double point);
    
    // 均值滤波（使用最近n个点，默认使用全部历史点）
    double meanFilter(int n = -1) const;
    
    // 中值滤波（使用最近n个点，默认使用全部历史点）
    double medianFilter(int n = -1) const;
    
    // 设置指数衰减滤波的衰减率（0 < alpha <= 1）
    void setExponentialAlpha(double alpha);
    
    // 获取当前指数衰减滤波值
    double getExponentialValue() const;
    
    // 清除历史数据
    void clearHistory();
    
    // 获取已添加点的数量
    int getPointCount() const;
    
    // 获取最大历史点数
    int getMaxHistory() const;

private:
    int max_history_;          // 最大历史点数
    std::deque<double> history_; // 历史数据点（使用deque便于维护固定长度）
    int point_count_;          // 已添加点的计数
    
    // 指数衰减滤波相关
    double exponential_value_; // 当前指数衰减值
    double exponential_alpha_; // 衰减率
    bool exponential_initialized_; // 指数滤波器是否已初始化
};

#endif // SIMPLE_DATA_FILTER_H