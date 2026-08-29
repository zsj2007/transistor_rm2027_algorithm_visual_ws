#include "EKF/SuperPowerTarget.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sp_ekf {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
}

Target::Target(const ArmorObservation& armor,
               double radius,
               int armor_num,
               const Eigen::VectorXd& P0_diag)
    : armor_num_(armor_num) {
    // SP 模型中装甲位置 = 中心 - r*[cos(angle), sin(angle)]，据此反推中心。
    const double center_x = armor.xyz[0] + radius * std::cos(armor.angle);
    const double center_y = armor.xyz[1] + radius * std::sin(armor.angle);
    const double center_z = armor.xyz[2];

    // SP 11 维状态顺序：x vx y vy z vz a w r l h。
    // a/w 为车体相位及角速度；r 为主半径，l/h 描述交替装甲的半径/高度差。
    Eigen::VectorXd x0(11);
    x0 << center_x, 0.0,
          center_y, 0.0,
          center_z, 0.0,
          armor.angle, 0.0,
          radius, 0.0, 0.0;

    const Eigen::MatrixXd P0 = P0_diag.asDiagonal();
    auto x_add = [](const Eigen::VectorXd& a,
                    const Eigen::VectorXd& b) -> Eigen::VectorXd {
        Eigen::VectorXd c = a + b;
        // 每次状态校正后将相位限制在主值区间，避免跨 pi 时出现不连续。
        while (c[6] > kPi) c[6] -= 2.0 * kPi;
        while (c[6] <= -kPi) c[6] += 2.0 * kPi;
        return c;
    };

    ekf_ = ExtendedKalmanFilter(x0, P0, x_add);
}

void Target::predict(double dt) {
    // 平移位置与速度、相位与角速度均采用匀速模型；r/l/h 视为随机常量。
    Eigen::MatrixXd F(11, 11);
    F << 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1,  0, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0,  1, dt,0, 0, 0, 0, 0, 0, 0,
         0, 0,  0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0,  0, 0, 1, dt,0, 0, 0, 0, 0,
         0, 0,  0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0,  0, 0, 0, 0, 1, dt,0, 0, 0,
         0, 0,  0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0,  0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0,  0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 1;

    // SP Target::predict() 的普通四装甲过程噪声强度。
    constexpr double v1 = 100.0;
    constexpr double v2 = 400.0;
    const double a = dt * dt * dt * dt / 4.0;
    const double b = dt * dt * dt / 2.0;
    const double c = dt * dt;

    // 对每个“位置-速度”二元组采用白噪声加速度离散化块 [dt^4/4, dt^3/2; dt^3/2, dt^2]。
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
    Q(0,0)=a*v1; Q(0,1)=b*v1; Q(1,0)=b*v1; Q(1,1)=c*v1;
    Q(2,2)=a*v1; Q(2,3)=b*v1; Q(3,2)=b*v1; Q(3,3)=c*v1;
    Q(4,4)=a*v1; Q(4,5)=b*v1; Q(5,4)=b*v1; Q(5,5)=c*v1;
    Q(6,6)=a*v2; Q(6,7)=b*v2; Q(7,6)=b*v2; Q(7,7)=c*v2;
    // h = x[10]，高度差加入很小的随机游走过程噪声，
    // 防止连续观测时 P_h 一直缩小到过度自信。
    // ps：量级很小没啥用，视频测试随机移动+旋转一直会h与z有问题，调整到0.25后虽然h不太稳，但是放在装甲板上1cm并不会打在外面
    // 实车还需测试
    constexpr double q_h = 2.5e-1;  // m^2 / s
    Q(10, 10) = q_h * dt;

    auto f = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
        Eigen::VectorXd prior = F * x;
        prior[6] = limitRad(prior[6]);
        return prior;
    };
    ekf_.predict(F, Q, f);
}

// 将有限的最小二乘角速度写入 EKF 的 w 状态，避免异常数值污染滤波器。
void Target::setAngularVelocity(double angular_velocity) {
    if (std::isfinite(angular_velocity)) {
        ekf_.x[7] = angular_velocity;
    }
}

TargetUpdateDebug Target::update(const ArmorObservation& armor) {
    TargetUpdateDebug debug;
    const int id = associateArmor(armor);
    const std::vector<Eigen::Vector4d> xyza_list = armorXyzaList();

    debug.matched_id = id;
    debug.armor_switched = (id != last_id_);
    debug.predicted_xyza = xyza_list[static_cast<std::size_t>(id)];
    debug.position_error =
        (armor.xyz - debug.predicted_xyza.head<3>()).norm();
    debug.angle_error =
        std::abs(limitRad(armor.angle - debug.predicted_xyza[3]));

    is_switch_ = debug.armor_switched;
    if (is_switch_) ++switch_count_;
    last_id_ = id;
    ++update_count_;

    updateYpda(armor, id);
    debug.nis = ekf_.last_nis;
    return debug;
}

TargetUpdateDebug Target::updatePair(
    const ArmorObservation& primary,
    const ArmorObservation& secondary,
    const PairUpdateConfig& config) {
    if (!config.enabled || armor_num_ != 4) {
        TargetUpdateDebug debug = update(primary);
        debug.pair_requested = true;
        debug.pair_status = config.enabled ? "UNSUPPORTED_TOPOLOGY"
                                           : "DISABLED";
        return debug;
    }

    const int primary_id = associateArmor(primary);
    const std::vector<Eigen::Vector4d> xyza_list = armorXyzaList();

    TargetUpdateDebug pair_debug;
    pair_debug.pair_requested = true;
    pair_debug.matched_id = primary_id;
    pair_debug.armor_switched = (primary_id != last_id_);
    pair_debug.predicted_xyza =
        xyza_list[static_cast<std::size_t>(primary_id)];
    pair_debug.position_error =
        (primary.xyz - pair_debug.predicted_xyza.head<3>()).norm();
    pair_debug.angle_error =
        std::abs(limitRad(primary.angle - pair_debug.predicted_xyza[3]));

    struct Candidate {
        int id = -1;
        double nis = std::numeric_limits<double>::infinity();
        double position_error = std::numeric_limits<double>::infinity();
        double angle_error = std::numeric_limits<double>::infinity();
    };
    Candidate best;
    bool passed_geometry_gate = false;

    const std::array<int, 2> adjacent_ids{
        (primary_id + 1) % armor_num_,
        (primary_id + armor_num_ - 1) % armor_num_,
    };

    for (const int secondary_id : adjacent_ids) {
        const Eigen::Vector4d predicted =
            xyza_list[static_cast<std::size_t>(secondary_id)];
        const double position_error =
            (secondary.xyz - predicted.head<3>()).norm();
        const double angle_error =
            std::abs(limitRad(secondary.angle - predicted[3]));
        if (position_error > config.max_secondary_position_error_m ||
            angle_error > config.max_secondary_angle_error_rad) {
            continue;
        }
        passed_geometry_gate = true;

        Eigen::VectorXd z(8);
        z.segment<4>(0) = measurementVector(primary);
        z.segment<4>(4) = measurementVector(secondary);

        Eigen::VectorXd predicted_z(8);
        predicted_z.segment<4>(0) =
            predictedMeasurement(ekf_.x, primary_id);
        predicted_z.segment<4>(4) =
            predictedMeasurement(ekf_.x, secondary_id);

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(8, 11);
        H.block(0, 0, 4, 11) = hJacobian(ekf_.x, primary_id);
        H.block(4, 0, 4, 11) = hJacobian(ekf_.x, secondary_id);

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(8, 8);
        R.block<4, 4>(0, 0) = measurementCovariance(
            primary, config.measurement_variance_scale,
            config.angle_variance_scale);
        R.block<4, 4>(4, 4) = measurementCovariance(
            secondary, config.measurement_variance_scale,
            config.angle_variance_scale);

        Eigen::VectorXd residual(8);
        residual.segment<4>(0) = measurementResidual(
            z.segment<4>(0), predicted_z.segment<4>(0));
        residual.segment<4>(4) = measurementResidual(
            z.segment<4>(4), predicted_z.segment<4>(4));

        const Eigen::MatrixXd innovation =
            H * ekf_.P * H.transpose() + R;
        const Eigen::LDLT<Eigen::MatrixXd> ldlt(innovation);
        if (ldlt.info() != Eigen::Success) continue;
        const Eigen::VectorXd solved = ldlt.solve(residual);
        if (ldlt.info() != Eigen::Success || !solved.allFinite()) continue;
        const double nis = residual.dot(solved);
        if (!std::isfinite(nis)) continue;

        if (nis < best.nis) {
            best.id = secondary_id;
            best.nis = nis;
            best.position_error = position_error;
            best.angle_error = angle_error;
        }
    }

    if (best.id < 0 || best.nis > config.max_joint_nis) {
        TargetUpdateDebug debug = update(primary);
        debug.pair_requested = true;
        debug.pair_used = false;
        debug.second_matched_id = best.id;
        debug.joint_nis = std::isfinite(best.nis) ? best.nis : -1.0;
        debug.second_position_error =
            std::isfinite(best.position_error) ? best.position_error : -1.0;
        debug.second_angle_error =
            std::isfinite(best.angle_error) ? best.angle_error : -1.0;
        debug.pair_status = !passed_geometry_gate
                                ? "SECONDARY_GEOMETRY_GATE"
                                : (best.id < 0 ? "JOINT_NUMERIC_FAILURE"
                                               : "JOINT_NIS_GATE");
        return debug;
    }

    is_switch_ = pair_debug.armor_switched;
    if (is_switch_) ++switch_count_;
    last_id_ = primary_id;
    ++update_count_;

    updateJointYpda(primary, primary_id, secondary, best.id, config);
    pair_debug.nis = ekf_.last_nis;
    pair_debug.pair_used = true;
    pair_debug.second_matched_id = best.id;
    pair_debug.joint_nis = best.nis;
    pair_debug.second_position_error = best.position_error;
    pair_debug.second_angle_error = best.angle_error;
    pair_debug.pair_status = "JOINT_OK";
    return pair_debug;
}

int Target::associateArmor(const ArmorObservation& armor) const {
    const std::vector<Eigen::Vector4d> xyza_list = armorXyzaList();
    std::vector<std::pair<Eigen::Vector4d, int>> candidates;
    candidates.reserve(static_cast<std::size_t>(armor_num_));
    for (int i = 0; i < armor_num_; ++i) {
        candidates.push_back(
            {xyza_list[static_cast<std::size_t>(i)], i});
    }

    std::sort(
        candidates.begin(), candidates.end(),
        [](const std::pair<Eigen::Vector4d, int>& lhs,
           const std::pair<Eigen::Vector4d, int>& rhs) {
            return lhs.first.head<3>().norm() <
                   rhs.first.head<3>().norm();
        });

    int id = 0;
    double min_angle_error = std::numeric_limits<double>::infinity();
    const Eigen::Vector3d armor_ypd = xyz2ypd(armor.xyz);
    const int inspect_count = std::min(3, armor_num_);
    for (int i = 0; i < inspect_count; ++i) {
        const Eigen::Vector4d& xyza =
            candidates[static_cast<std::size_t>(i)].first;
        const Eigen::Vector3d ypd = xyz2ypd(xyza.head<3>());
        const double angle_error =
            std::abs(limitRad(armor.angle - xyza[3])) +
            std::abs(limitRad(armor_ypd[0] - ypd[0]));
        if (angle_error < min_angle_error) {
            id = candidates[static_cast<std::size_t>(i)].second;
            min_angle_error = angle_error;
        }
    }
    return id;
}

Eigen::Vector4d Target::measurementVector(
    const ArmorObservation& armor) const {
    const Eigen::Vector3d ypd = xyz2ypd(armor.xyz);
    return {ypd[0], ypd[1], ypd[2], armor.angle};
}

Eigen::Matrix4d Target::measurementCovariance(
    const ArmorObservation& armor,
    double variance_scale,
    double angle_variance_scale) const {
    const double center_yaw = std::atan2(armor.xyz[1], armor.xyz[0]);
    const double delta_angle = limitRad(armor.angle - center_yaw);
    const Eigen::Vector3d ypd = xyz2ypd(armor.xyz);

    Eigen::Vector4d diagonal;
    diagonal << 4e-3,
                4e-3,
                std::log(std::abs(delta_angle) + 1.0) + 1.0,
                std::log(std::abs(ypd[2]) + 1.0) / 200.0 + 9e-2;
    diagonal *= std::max(variance_scale, 1e-6);
    diagonal[3] *= std::max(angle_variance_scale, 1e-6);
    return diagonal.asDiagonal();
}

Eigen::Vector4d Target::predictedMeasurement(
    const Eigen::VectorXd& x,
    int id) const {
    const Eigen::Vector3d xyz = armorXyz(x, id);
    const Eigen::Vector3d ypd = xyz2ypd(xyz);
    const double angle = limitRad(
        x[6] + id * 2.0 * kPi / static_cast<double>(armor_num_));
    return {ypd[0], ypd[1], ypd[2], angle};
}

Eigen::Vector4d Target::measurementResidual(
    const Eigen::Vector4d& measured,
    const Eigen::Vector4d& predicted) {
    Eigen::Vector4d residual = measured - predicted;
    residual[0] = limitRad(residual[0]);
    residual[1] = limitRad(residual[1]);
    residual[3] = limitRad(residual[3]);
    return residual;
}

void Target::updateYpda(const ArmorObservation& armor, int id) {
    const Eigen::MatrixXd H = hJacobian(ekf_.x, id);
    const Eigen::Vector4d z = measurementVector(armor);
    const Eigen::Matrix4d R =
        measurementCovariance(armor, 1.0, 1.0);

    auto h = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
        return predictedMeasurement(x, id);
    };
    auto z_subtract = [](const Eigen::VectorXd& a,
                         const Eigen::VectorXd& b) -> Eigen::VectorXd {
        return measurementResidual(a.head<4>(), b.head<4>());
    };
    ekf_.update(z, H, R, h, z_subtract);
}

void Target::updateJointYpda(
    const ArmorObservation& primary,
    int primary_id,
    const ArmorObservation& secondary,
    int secondary_id,
    const PairUpdateConfig& config) {
    Eigen::VectorXd z(8);
    z.segment<4>(0) = measurementVector(primary);
    z.segment<4>(4) = measurementVector(secondary);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(8, 11);
    H.block(0, 0, 4, 11) = hJacobian(ekf_.x, primary_id);
    H.block(4, 0, 4, 11) = hJacobian(ekf_.x, secondary_id);

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(8, 8);
    R.block<4, 4>(0, 0) = measurementCovariance(
        primary, config.measurement_variance_scale,
        config.angle_variance_scale);
    R.block<4, 4>(4, 4) = measurementCovariance(
        secondary, config.measurement_variance_scale,
        config.angle_variance_scale);

    auto h = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
        Eigen::VectorXd predicted(8);
        predicted.segment<4>(0) =
            predictedMeasurement(x, primary_id);
        predicted.segment<4>(4) =
            predictedMeasurement(x, secondary_id);
        return predicted;
    };
    auto z_subtract = [](const Eigen::VectorXd& a,
                         const Eigen::VectorXd& b) -> Eigen::VectorXd {
        Eigen::VectorXd residual(8);
        residual.segment<4>(0) = measurementResidual(
            a.segment<4>(0), b.segment<4>(0));
        residual.segment<4>(4) = measurementResidual(
            a.segment<4>(4), b.segment<4>(4));
        return residual;
    };

    ekf_.update(z, H, R, h, z_subtract,
                config.max_joint_nis, config.max_joint_nis);
}

Eigen::VectorXd Target::ekfX() const { return ekf_.x; }

const ExtendedKalmanFilter& Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armorXyzaList() const {
    std::vector<Eigen::Vector4d> result;
    result.reserve(static_cast<std::size_t>(armor_num_));
    // 按编号生成每块装甲的三维位置和外法线角预测值。
    for (int i = 0; i < armor_num_; ++i) {
        const double angle = limitRad(
            ekf_.x[6] + i * 2.0 * kPi / static_cast<double>(armor_num_));
        const Eigen::Vector3d xyz = armorXyz(ekf_.x, i);
        result.push_back({xyz[0], xyz[1], xyz[2], angle});
    }
    return result;
}

bool Target::diverged() const {
    // r 与 r+l 分别对应两组对置装甲半径，任一超出合理范围即视为发散。
    const bool r_ok = ekf_.x[8] > 0.1 && ekf_.x[8] < 0.4;
    const bool l_ok = (ekf_.x[8] + ekf_.x[9]) > 0.1 &&
                      (ekf_.x[8] + ekf_.x[9]) < 0.4;
    return !(r_ok && l_ok);
}

bool Target::converged() {
    // 至少经过四次量测更新且几何参数正常后，目标才可认为已收敛。
    if (update_count_ > 3 && !diverged()) {
        is_converged_ = true;
    }
    return is_converged_;
}

Eigen::Vector3d Target::armorXyz(const Eigen::VectorXd& x, int id) const {
    const double angle = limitRad(
        x[6] + id * 2.0 * kPi / static_cast<double>(armor_num_));
    // 四装甲模型中 1/3 号面使用另一组半径和高度偏置，其余使用基准 r/中心高度。
    const bool use_l_h = armor_num_ == 4 && (id == 1 || id == 3);
    const double r = use_l_h ? x[8] + x[9] : x[8];
    const double armor_x = x[0] - r * std::cos(angle);
    const double armor_y = x[2] - r * std::sin(angle);
    const double armor_z = use_l_h ? x[4] + x[10] : x[4];
    return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::hJacobian(const Eigen::VectorXd& x, int id) const {
    // 先构造状态到装甲 [x,y,z,angle] 的导数，再左乘 xyz 到 y/p/d 的坐标变换雅可比。
    const double angle = limitRad(
        x[6] + id * 2.0 * kPi / static_cast<double>(armor_num_));
    const bool use_l_h = armor_num_ == 4 && (id == 1 || id == 3);
    const double r = use_l_h ? x[8] + x[9] : x[8];

    const double dx_da = r * std::sin(angle);
    const double dy_da = -r * std::cos(angle);
    const double dx_dr = -std::cos(angle);
    const double dy_dr = -std::sin(angle);
    const double dx_dl = use_l_h ? -std::cos(angle) : 0.0;
    const double dy_dl = use_l_h ? -std::sin(angle) : 0.0;
    const double dz_dh = use_l_h ? 1.0 : 0.0;

    Eigen::MatrixXd H_armor_xyza = Eigen::MatrixXd::Zero(4, 11);
    H_armor_xyza(0,0)=1.0; H_armor_xyza(0,6)=dx_da;
    H_armor_xyza(0,8)=dx_dr; H_armor_xyza(0,9)=dx_dl;
    H_armor_xyza(1,2)=1.0; H_armor_xyza(1,6)=dy_da;
    H_armor_xyza(1,8)=dy_dr; H_armor_xyza(1,9)=dy_dl;
    H_armor_xyza(2,4)=1.0; H_armor_xyza(2,10)=dz_dh;
    H_armor_xyza(3,6)=1.0;

    const Eigen::Vector3d armor_xyz = armorXyz(x, id);
    const Eigen::Matrix3d H_armor_ypd = xyz2ypdJacobian(armor_xyz);

    Eigen::Matrix4d H_armor_ypda = Eigen::Matrix4d::Zero();
    H_armor_ypda.block<3,3>(0,0) = H_armor_ypd;
    H_armor_ypda(3,3) = 1.0;
    return H_armor_ypda * H_armor_xyza;
}

double Target::limitRad(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle <= -kPi) angle += 2.0 * kPi;
    return angle;
}

Eigen::Vector3d Target::xyz2ypd(const Eigen::Vector3d& xyz) {
    const double x = xyz[0];
    const double y = xyz[1];
    const double z = xyz[2];
    const double yaw = std::atan2(y, x);
    const double horizontal = std::sqrt(x * x + y * y);
    const double pitch = std::atan2(z, horizontal);
    const double distance = std::sqrt(x * x + y * y + z * z);
    return {yaw, pitch, distance};
}

Eigen::Matrix3d Target::xyz2ypdJacobian(const Eigen::Vector3d& xyz) {
    const double x = xyz[0];
    const double y = xyz[1];
    const double z = xyz[2];

    // 与 SP 相同的解析雅可比。极小下限仅处理目标恰在 z 轴上的坐标奇点，避免 NaN。
    const double xy2 = std::max(x * x + y * y, 1e-12);
    const double xy = std::sqrt(xy2);
    const double d2 = std::max(xy2 + z * z, 1e-12);
    const double d = std::sqrt(d2);

    const double dyaw_dx = -y / xy2;
    const double dyaw_dy = x / xy2;

    const double dpitch_dx = -(x * z) / (d2 * xy);
    const double dpitch_dy = -(y * z) / (d2 * xy);
    const double dpitch_dz = xy / d2;

    const double ddistance_dx = x / d;
    const double ddistance_dy = y / d;
    const double ddistance_dz = z / d;

    Eigen::Matrix3d J;
    J << dyaw_dx,      dyaw_dy,      0.0,
         dpitch_dx,    dpitch_dy,    dpitch_dz,
         ddistance_dx, ddistance_dy, ddistance_dz;
    return J;
}

}  // namespace sp_ekf
