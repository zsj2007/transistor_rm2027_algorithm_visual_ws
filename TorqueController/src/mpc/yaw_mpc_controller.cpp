#include "mpc/yaw_mpc_controller.h"

#include <algorithm>

YawMpcController::YawMpcController(RobotCommunication* comm, double dt_control, int N,
                                   double J, double tau_c, double b, double tau_d,
                                   double max_torque, double max_torque_rate,
                                   double Q, double R, double Rd, int max_iter,
                                   double integral_gain)
    : comm_(comm), dt_(dt_control), N_(N), integral_gain_(integral_gain),
      // 与实车脚本一致：MPC 的 dt_sim = dt_control（每个控制周期一个仿真子步）
      mpc_(dt_control, dt_control, J, tau_c, b, tau_d,
           max_torque, max_torque_rate, N, Q, R, Rd, max_iter)
{}

// 单目标 step：目标延迟缓冲（最多 N 个，延迟 dt*N）后求解
YawMpcController::Result YawMpcController::step(double target_yaw, bool integral_enable) {
    target_buf_.push_back(target_yaw);
    while (target_buf_.size() > static_cast<size_t>(N_)) target_buf_.pop_front();
    return solve(integral_enable);
}

// 整序列 step：直接用传入序列替换自身 target_buf_
// （取前 N_ 个；不足 N_ 个时用最后一个值填充到 N_ 个），后续处理相同
YawMpcController::Result YawMpcController::step(const std::vector<double>& target_buf,
                                               bool integral_enable) {
    target_buf_.clear();
    size_t n = target_buf.size();
    double last_val = (n > 0) ? target_buf.back() : 0.0;
    size_t count = std::min<size_t>(n, static_cast<size_t>(N_));
    for (size_t i = 0; i < count; ++i) target_buf_.push_back(target_buf[i]);
    while (target_buf_.size() < static_cast<size_t>(N_)) target_buf_.push_back(last_val);
    return solve(integral_enable);
}

// 公共求解：读融合状态 + 构造参考序列 + MPC 求解（不发送）
YawMpcController::Result YawMpcController::solve(bool integral_enable) {
    Result r;
    if (!comm_) return r;

    // ---- 1. 读取融合状态 ----
    auto fused = comm_->getFused();
    if (!fused.valid) return r;   // 融合未就绪

    const double theta     = fused.yaw_pos;           // yaw 关节解卷绕位置
    const double omega     = fused.yaw_rate;          // yaw 关节速度（高频）
    const double theta_imu = fused.imu_yaw_unwrapped; // IMU yaw 解卷绕（world 系）

    // 底盘角速度（MCU 原始数据；参考序列外推用）
    double chassis_imu_omega = 0.0;
    {
        auto data = comm_->getLatestData();
        if (data.mcu_valid) chassis_imu_omega = data.mcu_packet.chassis_imu_omega;
    }

    r.delayed_target = target_buf_.front();

    // ---- 2. 参考序列（与实车 Python 逻辑一致）----
    // ref[i] = 延迟目标[i] − ((theta_imu − theta) + (i+1)·dt·chassis_imu_omega)
    std::vector<double> ref;
    ref.reserve(N_);
    for (size_t i = 0; i < target_buf_.size(); ++i) {
        ref.push_back(target_buf_[i] -
                      ((theta_imu - theta) + (i + 1) * dt_ * chassis_imu_omega));
    }
    while (ref.size() < static_cast<size_t>(N_)) ref.push_back(ref.back());

    // ---- 3. MPC 求解（返回发送所需值 + 参考/预测序列，不发送）----
    auto mres = mpc_.step(theta, omega, ref);
    r.yaw_target_angle    = mres.theta;
    r.yaw_target_velocity = mres.omega;
    r.yaw_torque          = mres.torque;
    r.ref_sequence        = mpc_.lastRef();     // 本次参考（目标）序列（N 个）
    r.pred_sequence       = mpc_.lastPred();    // 本次预测位置序列（N 个，不含当前）

    // ---- 4. 积分补偿 ----
    // 上一步预测的第一步后位置（mres.theta）vs 这一步实际角度（theta）
    if (integral_enable) {
        // 第一次 step 无上一步预测，不计算积分增量
        if (has_prev_pred_) {
            integral_ += integral_gain_ * (prev_pred_pos_ - theta);
        }
        prev_pred_pos_ = mres.theta;
        has_prev_pred_ = true;
        // yaw_torque += 积分值后限位到最大力矩范围内再返回
        r.yaw_torque = std::clamp(mres.torque + integral_,
                                  -mpc_.maxTorque(), mpc_.maxTorque());
    } else {
        // 未启用：积分值清空为 0，yaw_torque 保持 MPC 结果
        integral_ = 0.0;
        prev_pred_pos_ = mres.theta;
        has_prev_pred_ = true;
    }

    return r;
}
