#pragma once

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>
#include <string>

namespace sp_ekf {

class ExtendedKalmanFilter {
public:
    // 当前后验状态与其协方差矩阵；由预测和量测更新原地维护。
    Eigen::VectorXd x;
    Eigen::MatrixXd P;

    ExtendedKalmanFilter() = default;

    ExtendedKalmanFilter(
        const Eigen::VectorXd& x0,
        const Eigen::MatrixXd& P0,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                      const Eigen::VectorXd&)> x_add =
            [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
                return a + b;
            });

    // 线性状态转移：x = F*x，P = F*P*F^T + Q。
    Eigen::VectorXd predict(const Eigen::MatrixXd& F,
                            const Eigen::MatrixXd& Q);

    Eigen::VectorXd predict(
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& Q,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f);

    // 线性观测更新。z_subtract 可替换为角度环绕相减等流形上的残差。
    Eigen::VectorXd update(
        const Eigen::VectorXd& z,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& R,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                      const Eigen::VectorXd&)> z_subtract =
            [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
                return a - b;
            },
        double nis_threshold = 0.711,
        double nees_threshold = 0.711);

    // 非线性观测更新：h 为观测函数，H 为其在当前状态处的雅可比矩阵。
    Eigen::VectorXd update(
        const Eigen::VectorXd& z,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& R,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                      const Eigen::VectorXd&)> z_subtract =
            [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
                return a - b;
            },
        double nis_threshold = 0.711,
        double nees_threshold = 0.711);

    // 对外导出的滤波健康度诊断量，不参与数据关联决策。
    std::map<std::string, double> data;
    // 固定窗口内的 NIS 超阈值标记，用于判断滤波是否长期失配。
    std::deque<int> recent_nis_failures{0};
    std::size_t window_size = 100;
    double last_nis = 0.0;

private:
    // 与状态同维的单位矩阵，以及用于处理角度状态环绕的状态加法器。
    Eigen::MatrixXd I_;
    std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                  const Eigen::VectorXd&)> x_add_;

    // 累积诊断计数，仅用于保持上游统计行为。
    int nees_count_ = 0;
    int nis_count_ = 0;
    int total_count_ = 0;
};

}  // namespace sp_ekf
