// PeriodicDataPredictor.cpp
#include "predictor/PeriodicDataPredictor.h"

PeriodicDataPredictor::PeriodicDataPredictor(int max_history, int fourier_order) 
    : max_history_(max_history), fourier_order_(fourier_order) {
    if (max_history <= 0) {
        throw std::invalid_argument("max_history must be positive");
    }
    if (fourier_order <= 0) {
        throw std::invalid_argument("fourier_order must be positive");
    }
    
    // 初始化系数向量
    a_coeffs_.resize(fourier_order, 0.0);
    b_coeffs_.resize(fourier_order, 0.0);
}

void PeriodicDataPredictor::addPoint(double point) {
    history_.push_back(point);
    point_count_++;
    
    // 保持历史不超过最大步数
    if (history_.size() > max_history_) {
        history_.erase(history_.begin());
    }
    
    coefficients_dirty_ = true;
}

void PeriodicDataPredictor::setPeriod(int period) {
    if (period <= 0) {
        period_ = 1;
    } else {
        period_ = period;
    }
    coefficients_dirty_ = true;
}

void PeriodicDataPredictor::setFourierOrder(int order) {
    if (order <= 0) {
        throw std::invalid_argument("Fourier order must be positive");
    }
    
    if (order != fourier_order_) {
        fourier_order_ = order;
        a_coeffs_.resize(order, 0.0);
        b_coeffs_.resize(order, 0.0);
        coefficients_dirty_ = true;
    }
}

int PeriodicDataPredictor::getFourierOrder() const {
    return fourier_order_;
}

void PeriodicDataPredictor::autoFindPeriod() {
    std::vector<double> modified_acf = computeModifiedACF(history_);

    std::vector<double> acf_stack = lagStackWithDecay(modified_acf);
    auto acf_stack_max_iter = std::max_element(acf_stack.begin(), acf_stack.end());
    period_ = std::max(static_cast<int>(std::distance(acf_stack.begin(), acf_stack_max_iter)), 1);
}

int PeriodicDataPredictor::getPeriod() const {
    return period_;
}

double PeriodicDataPredictor::smooth(int time_index) const {
    if (history_.empty()) {
        return 0.0;
    }
    
    // 如果需要，重新计算傅里叶系数
    if (coefficients_dirty_) {
        computeFourierCoefficients();
    }
    
    // 计算绝对时间索引
    // 正数表示相对最后添加的数据点向后预测
    // 负数表示相对最后一个数据点向前x索引处数据的平滑
    int absolute_index = static_cast<int>(history_.size()) - 1 + time_index;
    
    // 使用傅里叶级数计算平滑值（多阶）
    double t = static_cast<double>(absolute_index);
    double result = a0_;
    
    for (int k = 1; k <= fourier_order_; k++) {
        double omega_k = 2 * M_PI * k / period_;
        result += a_coeffs_[k-1] * std::cos(omega_k * t) + b_coeffs_[k-1] * std::sin(omega_k * t);
    }
    
    return result;
}

bool PeriodicDataPredictor::isRising(int time_index, double compare_threshold) const {
    // 计算导数并判断是否大于阈值
    return computeDerivative(time_index) > compare_threshold;
}

bool PeriodicDataPredictor::isUpper(int time_index, double compare_threshold) const {
    // 计算结果并判断是否大于阈值
    return smooth(time_index) > compare_threshold;
}

double PeriodicDataPredictor::getA0() const {
    return a0_;
}

double PeriodicDataPredictor::getCoefficientA(int order) const {
    if (order < 1 || order > fourier_order_) {
        throw std::out_of_range("Order out of range");
    }
    
    if (coefficients_dirty_) {
        computeFourierCoefficients();
    }
    
    return a_coeffs_[order-1];
}

double PeriodicDataPredictor::getCoefficientB(int order) const {
    if (order < 1 || order > fourier_order_) {
        throw std::out_of_range("Order out of range");
    }
    
    if (coefficients_dirty_) {
        computeFourierCoefficients();
    }
    
    return b_coeffs_[order-1];
}

void PeriodicDataPredictor::clearHistory() {
    history_.clear();
    point_count_ = 0;
    coefficients_dirty_ = true;
}

int PeriodicDataPredictor::getPointCount() const {
    return point_count_;
}

void PeriodicDataPredictor::computeFourierCoefficients() const {
    if (history_.empty()) {
        a0_ = 0.0;
        std::fill(a_coeffs_.begin(), a_coeffs_.end(), 0.0);
        std::fill(b_coeffs_.begin(), b_coeffs_.end(), 0.0);
        coefficients_dirty_ = false;
        return;
    }
    
    if (period_ <= 0) {
        throw std::runtime_error("Period must be set before computing Fourier coefficients");
    }
    
    int n = static_cast<int>(history_.size());
    
    // 计算a0 (直流分量)
    a0_ = std::accumulate(history_.begin(), history_.end(), 0.0) / n;
    
    // 重置系数
    std::fill(a_coeffs_.begin(), a_coeffs_.end(), 0.0);
    std::fill(b_coeffs_.begin(), b_coeffs_.end(), 0.0);
    
    // 计算各阶傅里叶系数
    for (int k = 1; k <= fourier_order_; k++) {
        double sum_cos = 0.0;
        double sum_sin = 0.0;
        
        for (int i = 0; i < n; i++) {
            double theta = 2 * M_PI * k * i / period_;
            sum_cos += history_[i] * std::cos(theta);
            sum_sin += history_[i] * std::sin(theta);
        }
        
        a_coeffs_[k-1] = 2.0 * sum_cos / n;
        b_coeffs_[k-1] = 2.0 * sum_sin / n;
    }
    
    coefficients_dirty_ = false;
}

double PeriodicDataPredictor::computeDerivative(int time_index) const {
    if (history_.empty() || period_ <= 0) {
        return 0.0;
    }
    
    // 如果需要，重新计算傅里叶系数
    if (coefficients_dirty_) {
        computeFourierCoefficients();
    }
    
    // 计算绝对时间索引
    int absolute_index = static_cast<int>(history_.size()) - 1 + time_index;
    double t = static_cast<double>(absolute_index);
    
    // 计算傅里叶级数的导数（多阶）
    // f(t) = a0 + Σ [ak*cos(kωt) + bk*sin(kωt)]
    // f'(t) = Σ [-ak*kω*sin(kωt) + bk*kω*cos(kωt)]
    double derivative = 0.0;
    for (int k = 1; k <= fourier_order_; k++) {
        double omega_k = 2 * M_PI * k / period_;
        derivative += -a_coeffs_[k-1] * omega_k * std::sin(omega_k * t) + 
                       b_coeffs_[k-1] * omega_k * std::cos(omega_k * t);
    }
    
    return derivative;
}

double PeriodicDataPredictor::getFitMse() {
    double result = 0.0;

    int n = static_cast<int>(history_.size());
    if (n == 0) {
        return 0.0;
    }
    for (int i = 0; i < n; i++) {
        double fit_data = smooth(-n + 1 + i);
        double error = fit_data - history_[i];
        result += error * error;
    }
    result /= static_cast<double>(n);

    return result;
};