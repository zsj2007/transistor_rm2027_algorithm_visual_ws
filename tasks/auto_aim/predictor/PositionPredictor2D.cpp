// PositionPredictor2D.cpp
#include "predictor/PositionPredictor2D.h"


PositionPredictor2D::PositionPredictor2D(int max_history) 
    : max_history_(max_history) {
    if (max_history <= 0) {
        throw std::invalid_argument("max_history must be positive");
    }
}

void PositionPredictor2D::addPoint(const cv::Point2f& point) {
    history_.push_back(point);
    point_count_++;
    
    // 保持历史不超过最大步数
    if (history_.size() > max_history_) {
        history_.erase(history_.begin());
    }
}

void PositionPredictor2D::fitLinear(int steps) {
    if (history_.empty()) {
        linear_fitted_ = false;
        return;
    }
    
    // 确定实际使用的步数
    int actual_steps = std::min(steps, static_cast<int>(history_.size()));
    
    // 提取最新的实际步数个点
    std::vector<cv::Point2f> recent_points(
        history_.end() - actual_steps, 
        history_.end()
    );
    
    // 提取x和y坐标
    std::vector<float> xs, ys;
    for (const auto& p : recent_points) {
        xs.push_back(p.x);
        ys.push_back(p.y);
    }
    
    // 分别拟合x和y方向
    linear_fit_x_ = fitLinearComponent(xs, point_count_, actual_steps);
    linear_fit_y_ = fitLinearComponent(ys, point_count_, actual_steps);

    last_mse_ = (linear_fit_x_.mse + linear_fit_y_.mse) / 2;
    
    linear_fitted_ = true;
}

// 修改后的 fitQuadratic 函数
void PositionPredictor2D::fitQuadratic(int steps) {
    if (history_.empty()) {
        quadratic_fitted_ = false;
        return;
    }
    
    // 确定实际使用的步数
    int actual_steps = std::min(steps, static_cast<int>(history_.size()));
    
    // 至少需要3个点进行二次拟合
    if (actual_steps < 3) {
        quadratic_fitted_ = false;
        return;
    }
    
    // 提取最新的实际步数个点
    std::vector<cv::Point2f> recent_points(
        history_.end() - actual_steps, 
        history_.end()
    );
    
    // 提取x和y坐标
    std::vector<float> xs, ys;
    for (const auto& p : recent_points) {
        xs.push_back(p.x);
        ys.push_back(p.y);
    }
    
    try {
        // 尝试进行二次拟合
        quadratic_fit_x_ = fitQuadraticComponent(xs, point_count_, actual_steps);
        quadratic_fit_y_ = fitQuadraticComponent(ys, point_count_, actual_steps);
        
        last_mse_ = (quadratic_fit_x_.mse + quadratic_fit_y_.mse);
        quadratic_fitted_ = true;
    } 
    catch (const std::runtime_error& e) {
        // 如果二次拟合失败，回退到线性拟合
        std::cerr << "Quadratic fit failed: " << e.what() 
                  << ". Falling back to linear fit." << std::endl;
        
        // 进行线性拟合
        fitLinear(steps);
        
        // 创建伪二次拟合结果（实际上是线性）
        quadratic_fit_x_.a0 = linear_fit_x_.b;
        quadratic_fit_x_.a1 = linear_fit_x_.a;
        quadratic_fit_x_.a2 = 0.0;
        quadratic_fit_x_.mse = linear_fit_x_.mse;
        quadratic_fit_x_.fit_point_count = point_count_;
        quadratic_fit_x_.fit_steps = actual_steps;
        
        quadratic_fit_y_.a0 = linear_fit_y_.b;
        quadratic_fit_y_.a1 = linear_fit_y_.a;
        quadratic_fit_y_.a2 = 0.0;
        quadratic_fit_y_.mse = linear_fit_y_.mse;
        quadratic_fit_y_.fit_point_count = point_count_;
        quadratic_fit_y_.fit_steps = actual_steps;
        
        last_mse_ = (linear_fit_x_.mse + linear_fit_y_.mse);
        quadratic_fitted_ = true;  // 标记为已拟合
    }
}

void PositionPredictor2D::fitFourier(int steps, int fourier_order) {
    if (history_.empty()) {
        fourier_fitted_ = false;
        return;
    }
    
    // 检查是否需要重新进行线性拟合
    bool need_refit_linear = false;
    if (!linear_fitted_ || 
        linear_fit_x_.fit_point_count != point_count_ ||
        linear_fit_x_.fit_steps != steps) {
        need_refit_linear = true;
    }
    
    // 如果需要重新拟合线性分量
    if (need_refit_linear) {
        fitLinear(steps);
        if (!linear_fitted_) {
            fourier_fitted_ = false;
            return;
        }
    }
    
    // 确定实际使用的步数
    int actual_steps = std::min(steps, static_cast<int>(history_.size()));
    
    // 提取最新的实际步数个点
    std::vector<cv::Point2f> recent_points(
        history_.end() - actual_steps, 
        history_.end()
    );
    
    // 提取x和y坐标
    std::vector<float> xs, ys;
    for (const auto& p : recent_points) {
        xs.push_back(p.x);
        ys.push_back(p.y);
    }
    
    // 获取线性拟合后的残差
    std::vector<double> res_x = linear_fit_x_.residuals;
    std::vector<double> res_y = linear_fit_y_.residuals;
    
    // 计算Modified ACF并找到周期
    auto modified_acf_x = computeModifiedACF(res_x);
    auto modified_acf_y = computeModifiedACF(res_y);
    
    // 取两个方向ACF的和
    std::vector<double> modified_acf_combined(modified_acf_x.size());
    for (size_t i = 0; i < modified_acf_x.size(); i++) {
        modified_acf_combined[i] = (modified_acf_x[i] + modified_acf_y[i]);
    }
    
    std::vector<double> acf_stack = lagStackWithDecay(modified_acf_combined);
    auto acf_stack_max_iter = std::max_element(acf_stack.begin(), acf_stack.end());
    int T = std::distance(acf_stack.begin(), acf_stack_max_iter);
    if (T <= 0) T = 1;  // 确保周期有效
    
    // 拟合傅里叶级数
    fourier_fit_.beta_x = fitFourierSeries(res_x, T, fourier_order);
    fourier_fit_.beta_y = fitFourierSeries(res_y, T, fourier_order);
    fourier_fit_.period = T;
    fourier_fit_.fit_point_count = point_count_;
    fourier_fit_.fit_steps = actual_steps;
    
    // 计算拟合MSE
    double mse_x = 0.0;
    double mse_y = 0.0;
    for (int t = 0; t < actual_steps; t++) {
        double pred_x = linear_fit_x_.a * t + linear_fit_x_.b + 
                       fourierSeries(t, fourier_fit_.beta_x, T, fourier_order);
        double pred_y = linear_fit_y_.a * t + linear_fit_y_.b + 
                       fourierSeries(t, fourier_fit_.beta_y, T, fourier_order);
        
        mse_x += std::pow(pred_x - xs[t], 2);
        mse_y += std::pow(pred_y - ys[t], 2);
    }
    fourier_fit_.mse = (mse_x + mse_y) / (actual_steps);

    last_mse_ = fourier_fit_.mse;
    
    fourier_fitted_ = true;
}

std::vector<cv::Point2f> PositionPredictor2D::predictLinear(int pred_length, cv::Point2f ground_stable_point) const {
    std::vector<cv::Point2f> predictions;
    
    if (history_.empty()) {
        // 没有历史数据，返回固定值 (0,0)
        for (int i = 0; i < pred_length; i++) {
            predictions.push_back(cv::Point2f(0, 0) + ground_stable_point);
        }
        return predictions;
    }
    if (!linear_fitted_) {
        // 未拟合，返回最近的值
        for (int i = 0; i < pred_length; i++) {
            predictions.push_back(history_.back() + ground_stable_point);
        }
        return predictions;
    }
    
    int start_t = history_.size();
    for (int i = 0; i < pred_length; i++) {
        double t = start_t + i;
        float x = linear_fit_x_.a * t + linear_fit_x_.b;
        float y = linear_fit_y_.a * t + linear_fit_y_.b;
        predictions.push_back(cv::Point2f(x, y) + ground_stable_point);
    }
    
    return predictions;
}

// 新增：二次曲线预测函数实现
std::vector<cv::Point2f> PositionPredictor2D::predictQuadratic(int pred_length, cv::Point2f ground_stable_point) const {
    std::vector<cv::Point2f> predictions;
    
    if (history_.empty()) {
        // 没有历史数据，返回固定值 (0,0)
        for (int i = 0; i < pred_length; i++) {
            predictions.push_back(cv::Point2f(0, 0) + ground_stable_point);
        }
        return predictions;
    }
    if (!quadratic_fitted_) {
        // 未拟合，返回最近的值
        for (int i = 0; i < pred_length; i++) {
            predictions.push_back(history_.back() + ground_stable_point);
        }
        return predictions;
    }

    int start_t = history_.size();
    for (int i = 0; i < pred_length; i++) {
        double t = start_t + i;
        float x = quadratic_fit_x_.a0 + quadratic_fit_x_.a1 * t + quadratic_fit_x_.a2 * t * t;
        float y = quadratic_fit_y_.a0 + quadratic_fit_y_.a1 * t + quadratic_fit_y_.a2 * t * t;
        predictions.push_back(cv::Point2f(x, y) + ground_stable_point);
    }
    
    return predictions;
}

std::vector<cv::Point2f> PositionPredictor2D::predictFourier(int pred_length, cv::Point2f ground_stable_point) const {
    std::vector<cv::Point2f> predictions;
    

    if (history_.empty()) {
        // 没有历史数据，返回固定值 (0,0)
        for (int i = 0; i < pred_length; i++) {
            predictions.push_back(cv::Point2f(0, 0) + ground_stable_point);
        }
        return predictions;
    }
    if (!fourier_fitted_) {
        // 未拟合，返回最近的值
        for (int i = 0; i < pred_length; i++) {
            predictions.push_back(history_.back() + ground_stable_point);
        }
        return predictions;
    }
    
    int start_t = history_.size();
    int T = fourier_fit_.period;
    int N = (fourier_fit_.beta_x.size() - 1) / 2;  // 计算傅里叶阶数
    
    for (int i = 0; i < pred_length; i++) {
        double t = start_t + i;
        float x = linear_fit_x_.a * t + linear_fit_x_.b + 
                 fourierSeries(t, fourier_fit_.beta_x, T, N);
        float y = linear_fit_y_.a * t + linear_fit_y_.b + 
                 fourierSeries(t, fourier_fit_.beta_y, T, N);
        predictions.push_back(cv::Point2f(x, y) + ground_stable_point);
    }
    
    return predictions;
}

double PositionPredictor2D::getLastMSE() const {
    return last_mse_;
}

void PositionPredictor2D::clearHistory() {
    history_.clear();
    point_count_ = 0;
    linear_fitted_ = false;
    quadratic_fitted_ = false;  // 新增
    fourier_fitted_ = false;
}

int PositionPredictor2D::getPointCount() const {
    return point_count_;
}

cv::Vec2f PositionPredictor2D::getLinearVelocity() const {
    if (!linear_fitted_) {
        // 未进行线性拟合，返回零向量
        return cv::Vec2f(0.0f, 0.0f);
    }
    // 返回速度向量 (v_x, v_y) = (a_x, a_y)
    return cv::Vec2f(
        static_cast<float>(linear_fit_x_.a), 
        static_cast<float>(linear_fit_y_.a)
    );
}

// 私有方法实现
PositionPredictor2D::LinearFitResult PositionPredictor2D::fitLinearComponent(
    const std::vector<float>& y, int point_count, int steps) const 
{
    LinearFitResult result;
    result.fit_point_count = point_count;
    result.fit_steps = steps;
    
    int n = y.size();
    std::vector<double> t(n);
    for (int i = 0; i < n; i++) {
        t[i] = i;
    }
    
    // 计算均值
    double t_mean = std::accumulate(t.begin(), t.end(), 0.0) / n;
    double y_mean = std::accumulate(y.begin(), y.end(), 0.0) / n;
    
    // 计算协方差和方差
    double covariance = 0.0;
    double var_t = 0.0;
    for (int i = 0; i < n; i++) {
        covariance += (t[i] - t_mean) * (y[i] - y_mean);
        var_t += (t[i] - t_mean) * (t[i] - t_mean);
    }
    
    // 计算斜率和截距
    result.a = (var_t != 0) ? covariance / var_t : 0.0;
    result.b = y_mean - result.a * t_mean;
    
    // 计算线性分量和残差
    result.residuals.resize(n);
    double mse = 0.0;
    for (int i = 0; i < n; i++) {
        double linear = result.a * t[i] + result.b;
        result.residuals[i] = y[i] - linear;
        mse += result.residuals[i] * result.residuals[i];
    }
    result.mse = mse / n;
    
    return result;
}

// 修改后的 fitQuadraticComponent 函数
PositionPredictor2D::QuadraticFitResult PositionPredictor2D::fitQuadraticComponent(
    const std::vector<float>& y, int point_count, int steps) const 
{
    QuadraticFitResult result;
    result.fit_point_count = point_count;
    result.fit_steps = steps;
    
    int n = y.size();
    if (n < 3) {
        throw std::runtime_error("At least 3 points are required for quadratic fitting");
    }
    
    std::vector<double> t(n);
    for (int i = 0; i < n; i++) {
        t[i] = i;
    }
    
    // 计算各项和
    double sum_t = 0.0, sum_t2 = 0.0, sum_t3 = 0.0, sum_t4 = 0.0;
    double sum_y = 0.0, sum_ty = 0.0, sum_t2y = 0.0;
    
    for (int i = 0; i < n; i++) {
        double ti = t[i];
        double ti2 = ti * ti;
        double ti3 = ti2 * ti;
        double ti4 = ti2 * ti2;
        double yi = y[i];
        
        sum_t += ti;
        sum_t2 += ti2;
        sum_t3 += ti3;
        sum_t4 += ti4;
        sum_y += yi;
        sum_ty += ti * yi;
        sum_t2y += ti2 * yi;
    }
    
    // 构建正规方程矩阵
    double S00 = n;
    double S10 = sum_t;
    double S20 = sum_t2;
    double S01 = sum_y;
    double S11 = sum_ty;
    double S21 = sum_t2y;
    
    double S30 = sum_t3;
    double S40 = sum_t4;
    
    // 计算行列式
    double det = S00 * (S20 * S40 - S30 * S30) - 
                 S10 * (S10 * S40 - S30 * S20) + 
                 S20 * (S10 * S30 - S20 * S20);
    
    // 检查行列式是否接近零
    if (std::abs(det) < 1e-10) {
        throw std::runtime_error("Singular matrix in quadratic fitting (det ≈ 0)");
    }
    
    // 计算逆矩阵
    double inv00 = (S20 * S40 - S30 * S30) / det;
    double inv01 = (S30 * S20 - S10 * S40) / det;
    double inv02 = (S10 * S30 - S20 * S20) / det;
    
    double inv10 = (S30 * S20 - S10 * S40) / det;
    double inv11 = (S00 * S40 - S20 * S20) / det;
    double inv12 = (S20 * S10 - S00 * S30) / det;
    
    double inv20 = (S10 * S30 - S20 * S20) / det;
    double inv21 = (S20 * S10 - S00 * S30) / det;
    double inv22 = (S00 * S20 - S10 * S10) / det;
    
    // 计算系数
    result.a0 = inv00 * S01 + inv01 * S11 + inv02 * S21;
    result.a1 = inv10 * S01 + inv11 * S11 + inv12 * S21;
    result.a2 = inv20 * S01 + inv21 * S11 + inv22 * S21;
    
    // 计算MSE
    double mse = 0.0;
    for (int i = 0; i < n; i++) {
        double pred = result.a0 + result.a1 * t[i] + result.a2 * t[i] * t[i];
        double error = y[i] - pred;
        mse += error * error;
    }
    result.mse = mse / n;
    
    return result;
}

std::vector<double> PositionPredictor2D::fitFourierSeries(const std::vector<double>& residuals, int T, int N) const {
    if (T <= 0) {
        throw std::invalid_argument("Period T must be positive");
    }
    
    int L = residuals.size();
    if (L == 0) return {};
    
    std::vector<double> beta;
    // a0
    double a0 = std::accumulate(residuals.begin(), residuals.end(), 0.0) / L;
    beta.push_back(a0);
    
    for (int n = 1; n <= N; n++) {
        double sum_cos = 0.0;
        double sum_sin = 0.0;
        
        for (int t = 0; t < L; t++) {
            double theta = 2 * M_PI * n * t / T;
            sum_cos += residuals[t] * std::cos(theta);
            sum_sin += residuals[t] * std::sin(theta);
        }
        
        double an = (2.0 / L) * sum_cos;
        double bn = (2.0 / L) * sum_sin;
        beta.push_back(an);
        beta.push_back(bn);
    }
    
    return beta;
}

double PositionPredictor2D::fourierSeries(double t, const std::vector<double>& beta, int T, int N) const {
    if (beta.empty()) return 0.0;
    
    double value = beta[0]; // a0
    for (int n = 1; n <= N; n++) {
        int idx = 1 + 2 * (n - 1);
        if (idx + 1 >= static_cast<int>(beta.size())) break;
        
        double a_n = beta[idx];
        double b_n = beta[idx + 1];
        double angle = 2 * M_PI * n * t / T;
        value += a_n * std::cos(angle) + b_n * std::sin(angle);
    }
    return value;
}