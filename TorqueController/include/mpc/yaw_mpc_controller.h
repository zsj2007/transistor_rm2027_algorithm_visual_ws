#ifndef YAW_MPC_CONTROLLER_H
#define YAW_MPC_CONTROLLER_H

#include <deque>
#include <vector>
#include "communication/Communications.hpp"
#include "mpc/mpc_controller.h"

// ============================================================================
// YawMpcController — 实车 yaw 轴 MPC 求解封装（参考序列 + 求解，不发送）
//
// - 初始化: 控制周期 dt_control、预测步数 N、MPC 参数（辨识 + 约束 + 权重）、
//   积分比例系数 integral_gain（默认 0.01）
// - 每步调用 step(target_yaw, integral_enable)（只传目标位置），内部完成:
//     1) 读取融合状态（yaw_pos / yaw_rate / imu_yaw_unwrapped）与
//        MCU 的 chassis_imu_omega（底盘角速度，参考外推用）
//     2) 维护延迟参考序列: 目标延迟 dt_control*N（buffer 保持最多 N 个），
//        ref[i] = 延迟目标[i] − ((theta_imu − theta) + (i+1)·dt·chassis_imu_omega)
//     3) MPC 求解第一步控制量
//     4) 积分补偿（integral_enable=true 时）:
//        存储上一步预测的第一步后位置，积分值 += integral_gain * (上一步预测值 −
//        这一步实际角度)（第一次 step 无上一步预测，不计算），返回的 yaw_torque
//        += 积分值后再限位到 ±max_torque 返回；false 时积分值清空为 0。
// - 返回将要发送的值（yaw_target_angle / yaw_target_velocity / yaw_torque），
//   由上层封装决定如何发送
// ============================================================================
class YawMpcController {
public:
    struct Result {
        double yaw_target_angle = 0.0;    // 预测位置 → 发送 yaw_target_angle (rad)
        double yaw_target_velocity = 0.0; // 预测速度 → 发送 yaw_target_velocity (rad/s)
        double yaw_torque = 0.0;          // 控制力矩 → 发送 yaw_torque (N·m)
        double delayed_target = 0.0;      // 当前参考（延迟 N 步的目标），供显示
        std::vector<double> ref_sequence;   // 本次参考（目标）序列（N 个）
        std::vector<double> pred_sequence;  // 本次预测位置序列（N 个，不含当前）
    };

    // dt_control: 控制周期（MPC 的 dt_control 与 dt_sim 相同）
    // N: 预测步数
    // integral_gain: 积分比例系数（默认 0.01）
    YawMpcController(RobotCommunication* comm, double dt_control, int N,
                     double J, double tau_c, double b, double tau_d,
                     double max_torque, double max_torque_rate,
                     double Q, double R, double Rd, int max_iter,
                     double integral_gain = 0.01);

    // 每步调用：内部读状态、维护延迟参考序列、求解；返回发送所需值（不发送）
    // integral_enable=true 时启用积分补偿（第一次 step 无上一步预测，不计算）：
    //   积分值 += integral_gain * (上一步预测的第一步后位置 − 这一步实际角度)；
    //   yaw_torque += 积分值后再限位到 ±max_torque 返回。
    // integral_enable=false 时积分值清空为 0，yaw_torque 不变。
    Result step(double target_yaw, bool integral_enable = false);

    // 整序列 step：直接用传入的目标缓冲序列替换内部 target_buf_
    // （取前 N_ 个，不足用最后一个值填充到 N_ 个），后续处理与 step(target_yaw) 相同
    Result step(const std::vector<double>& target_buf, bool integral_enable = false);

    // 当前积分值（积分补偿累积量；仅在积分补偿启用时增长，禁用时被清空为 0）
    double integral() const { return integral_; }

private:
    Result solve(bool integral_enable);   // 公共求解：读融合状态 + 参考序列 + MPC + 积分补偿

    RobotCommunication* comm_;
    double dt_;
    int    N_;
    double integral_gain_;        // 积分比例系数
    MPCController mpc_;
    std::deque<double> target_buf_;   // 延迟目标序列（最多 N 个，最旧在前）

    // 积分补偿状态
    double integral_ = 0.0;       // 积分值
    double prev_pred_pos_ = 0.0;  // 上一步预测的第一步后位置
    bool   has_prev_pred_ = false; // 是否已有上一步预测（第一次 step 不计算）
};

#endif // YAW_MPC_CONTROLLER_H
