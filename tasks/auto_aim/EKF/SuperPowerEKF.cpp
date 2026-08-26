#include "EKF/SuperPowerEKF.h"

#include <numeric>
#include <utility>

namespace sp_ekf {

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::VectorXd& x0,
    const Eigen::MatrixXd& P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                  const Eigen::VectorXd&)> x_add)
    : x(x0),
      P(P0),
      I_(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())),
      x_add_(std::move(x_add)) {
    // 预置所有诊断字段，使调用方即使在首次更新前也能安全读取。
    data["residual_yaw"] = 0.0;
    data["residual_pitch"] = 0.0;
    data["residual_distance"] = 0.0;
    data["residual_angle"] = 0.0;
    data["nis"] = 0.0;
    data["nees"] = 0.0;
    data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd& F,
                                               const Eigen::MatrixXd& Q) {
    return predict(F, Q, [&](const Eigen::VectorXd& value) {
        return F * value;
    });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f) {
    // 先传播不确定度，再传播状态；非线性模型由 f 负责状态本身的演化。
    P = F * P * F.transpose() + Q;
    x = f(x);
    return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
    const Eigen::VectorXd& z,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                  const Eigen::VectorXd&)> z_subtract,
    double nis_threshold,
    double nees_threshold) {
    return update(z, H, R,
                  [&](const Eigen::VectorXd& value) { return H * value; },
                  std::move(z_subtract), nis_threshold, nees_threshold);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
    const Eigen::VectorXd& z,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&,
                                  const Eigen::VectorXd&)> z_subtract,
    double nis_threshold,
    double nees_threshold) {
    // 保存更新前状态，以便计算本次校正幅度的 NEES 诊断量。
    const Eigen::VectorXd x_prior = x;
    // S = HPH^T + R 为创新协方差，K 将观测残差映射到状态校正量。
    Eigen::MatrixXd K =
        P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // Joseph 形式的协方差更新能更好保持 P 的对称性和半正定性，与上游 SP 一致。
    P = (I_ - K * H) * P * (I_ - K * H).transpose() + K * R * K.transpose();
    x = x_add_(x, K * z_subtract(z, h(x)));

    // SP 的更新后健康度统计。是诊断/复位信号，不是额外的数据关联门。
    const Eigen::VectorXd residual = z_subtract(z, h(x));
    const Eigen::MatrixXd S = H * P * H.transpose() + R;
    const double nis = residual.transpose() * S.inverse() * residual;
    const double nees =
        (x - x_prior).transpose() * P.inverse() * (x - x_prior);

    if (nis > nis_threshold) {
        ++nis_count_;
        data["nis_fail"] = 1.0;
    }
    if (nees > nees_threshold) {
        ++nees_count_;
        data["nees_fail"] = 1.0;
    }
    ++total_count_;
    last_nis = nis;

    // 维护固定长度滑窗，供 Tracker 判断长期不收敛。
    recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);
    if (recent_nis_failures.size() > window_size) {
        recent_nis_failures.pop_front();
    }

    const int recent_failures =
        std::accumulate(recent_nis_failures.begin(),
                        recent_nis_failures.end(), 0);
    const double recent_rate = static_cast<double>(recent_failures) /
        static_cast<double>(recent_nis_failures.size());

    data["residual_yaw"] = residual[0];
    data["residual_pitch"] = residual[1];
    data["residual_distance"] = residual[2];
    data["residual_angle"] = residual[3];
    data["nis"] = nis;
    data["nees"] = nees;
    data["recent_nis_failures"] = recent_rate;

    return x;
}

}  // namespace sp_ekf
