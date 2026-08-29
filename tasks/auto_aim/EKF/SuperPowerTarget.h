#pragma once

#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

#include "EKF/SuperPowerEKF.h"

// 算法内部类型：统一遵循 SuperPower 约定——米、秒、弧度；装甲 angle 为
// SP 定义的外法线 yaw，而非项目层直接使用的 yaw。
namespace sp_ekf {

struct ArmorObservation {
    // 单块装甲的三维位置和外法线角观测。
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    double angle = 0.0;
};

struct PairUpdateConfig {
    bool enabled = false;
    double max_joint_nis = 20.09;
    double max_secondary_position_error_m = 0.45;
    double max_secondary_angle_error_rad = 0.80;
    double measurement_variance_scale = 1.5;
    double angle_variance_scale = 4.0;
};

struct TargetUpdateDebug {
    // 一次量测更新的关联结果及更新前预测误差，用于诊断而非参与滤波计算。
    int matched_id = -1;
    bool armor_switched = false;
    Eigen::Vector4d predicted_xyza = Eigen::Vector4d::Zero();
    double position_error = -1.0;
    double angle_error = -1.0;
    double nis = -1.0;
    bool pair_requested = false;
    bool pair_used = false;
    int second_matched_id = -1;
    double joint_nis = -1.0;
    double second_position_error = -1.0;
    double second_angle_error = -1.0;
    std::string pair_status = "SINGLE";
};

class Target {
public:
    Target() = default;
    Target(const ArmorObservation& armor,
           double radius,
           int armor_num,
           const Eigen::VectorXd& P0_diag);

    // 按 dt 执行匀速位置和匀角速 yaw 的时间预测。
    void predict(double dt);
    // 使用滑窗最小二乘拟合值覆盖角速度 w，其他后验状态保持不变。
    void setAngularVelocity(double angular_velocity);
    // 为观测选择最匹配的装甲编号，再执行 EKF 量测更新。
    TargetUpdateDebug update(const ArmorObservation& armor);
    // 同一时间戳的双板联合更新。主板沿用现有关联，副板只枚举相邻拓扑；
    // 门控失败时在已经完成的本帧预测上回退为主板单观测更新。
    TargetUpdateDebug updatePair(
        const ArmorObservation& primary,
        const ArmorObservation& secondary,
        const PairUpdateConfig& config);

    // 直接暴露内部状态/滤波器仅供适配层导出和健康度检查。
    Eigen::VectorXd ekfX() const;
    const ExtendedKalmanFilter& ekf() const;
    std::vector<Eigen::Vector4d> armorXyzaList() const;

    // 半径是否越出物理可接受范围；converged 在足够有效更新后置位。
    bool diverged() const;
    bool converged();
    int lastId() const { return last_id_; }
    int armorNum() const { return armor_num_; }

private:
    // 关联和收敛状态：last_id_ 是上次匹配装甲，switch_count_ 记录切换次数。
    int armor_num_ = 4;
    int switch_count_ = 0;
    int update_count_ = 0;
    int last_id_ = 0;
    bool is_switch_ = false;
    bool is_converged_ = false;

    // 11 维车辆状态的 EKF。
    ExtendedKalmanFilter ekf_;

    // 在 y/p/d/angle 观测空间完成更新；其余函数描述几何观测模型及其雅可比。
    void updateYpda(const ArmorObservation& armor, int id);
    void updateJointYpda(
        const ArmorObservation& primary,
        int primary_id,
        const ArmorObservation& secondary,
        int secondary_id,
        const PairUpdateConfig& config);
    int associateArmor(const ArmorObservation& armor) const;
    Eigen::Vector4d measurementVector(const ArmorObservation& armor) const;
    Eigen::Matrix4d measurementCovariance(
        const ArmorObservation& armor,
        double variance_scale,
        double angle_variance_scale) const;
    Eigen::Vector4d predictedMeasurement(
        const Eigen::VectorXd& x,
        int id) const;
    static Eigen::Vector4d measurementResidual(
        const Eigen::Vector4d& measured,
        const Eigen::Vector4d& predicted);
    Eigen::Vector3d armorXyz(const Eigen::VectorXd& x, int id) const;
    Eigen::MatrixXd hJacobian(const Eigen::VectorXd& x, int id) const;

    // 角度归一化到 (-pi, pi]，并提供 xyz 到 yaw/pitch/distance 的变换及雅可比。
    static double limitRad(double angle);
    static Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz);
    static Eigen::Matrix3d xyz2ypdJacobian(const Eigen::Vector3d& xyz);
};

}  // namespace sp_ekf
