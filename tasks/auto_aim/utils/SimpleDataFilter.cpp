// SimpleDataFilter.cpp
#include "utils/SimpleDataFilter.h"

SimpleDataFilter::SimpleDataFilter(int max_history) 
    : max_history_(max_history), 
      point_count_(0),
      exponential_value_(0.0),
      exponential_alpha_(0.1),
      exponential_initialized_(false) {
    if (max_history <= 0) {
        throw std::invalid_argument("max_history must be positive");
    }
}

void SimpleDataFilter::addPoint(double point) {
    // 添加到历史数据
    history_.push_back(point);
    point_count_++;
    
    // 保持历史不超过最大点数
    if (history_.size() > max_history_) {
        history_.pop_front();
    }
    
    // 更新指数衰减滤波值
    if (!exponential_initialized_ || !(exponential_value_ < 1e100 && exponential_value_ > -1e100)) {
        exponential_value_ = point;
        exponential_initialized_ = true;
    } else {
        exponential_value_ = exponential_alpha_ * point + (1 - exponential_alpha_) * exponential_value_;
    }
}

double SimpleDataFilter::meanFilter(int n) const {
    if (history_.empty()) {
        return 0.0;
    }
    
    // 确定实际使用的点数
    int actual_n = n;
    if (n <= 0 || n > history_.size()) {
        actual_n = history_.size();
    }
    
    // 计算最近actual_n个点的均值
    auto start = history_.end() - actual_n;
    double sum = std::accumulate(start, history_.end(), 0.0);
    return sum / actual_n;
}

double SimpleDataFilter::medianFilter(int n) const {
    if (history_.empty()) {
        return 0.0;
    }
    
    // 确定实际使用的点数
    int actual_n = n;
    if (n <= 0 || n > history_.size()) {
        actual_n = history_.size();
    }
    
    // 复制最近actual_n个点
    auto start = history_.end() - actual_n;
    std::vector<double> recent_points(start, history_.end());
    
    // 排序并返回中值
    std::sort(recent_points.begin(), recent_points.end());
    
    if (actual_n % 2 == 0) {
        // 偶数个点，取中间两个的平均值
        return (recent_points[actual_n/2 - 1] + recent_points[actual_n/2]) / 2.0;
    } else {
        // 奇数个点，取中间值
        return recent_points[actual_n/2];
    }
}

void SimpleDataFilter::setExponentialAlpha(double alpha) {
    if (alpha <= 0.0 || alpha > 1.0) {
        throw std::invalid_argument("Alpha must be in range (0, 1]");
    }
    exponential_alpha_ = alpha;
}

double SimpleDataFilter::getExponentialValue() const {
    return exponential_value_;
}

void SimpleDataFilter::clearHistory() {
    history_.clear();
    point_count_ = 0;
    exponential_initialized_ = false;
    exponential_value_ = 0.0;
}

int SimpleDataFilter::getPointCount() const {
    return point_count_;
}

int SimpleDataFilter::getMaxHistory() const {
    return max_history_;
}