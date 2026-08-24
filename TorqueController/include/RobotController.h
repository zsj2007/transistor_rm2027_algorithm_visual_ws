#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <vector>
#include <cstdint>
#include <stdexcept>
#include "communication/Communications.hpp"
#include "mpc/mcu_mpc_controller.h"

// ============================================================================
// RobotController — 一体化控制封装
//
// 内部完成：
//   - 通信接口建立（RobotCommunication：MCU + IMU 串口 + 融合滤波器）
//   - yaw MPC 控制封装建立（McuMpcController，含 YawMpcController 与
//     后台 100Hz 发送线程）
//
// 外部用法：实例化一个 RobotController 即可完全控制——
//   - getState()：统一获取一个按来源分组的结构体（mcu / imu 原始数据、
//     FusionFilter 输出、mpc 状态；mpc 额外含最新一次运算的目标位置序列
//     ref_sequence（N 个）与预测位置序列 pred_sequence（N 个，不含当前））
//   - set()：直通 McuMpcController::set（设置发送参数 + mpc 目标）
// ============================================================================
class RobotController {
public:
    // ── 按来源分组的数据结构 ──

    // MCU 原始数据（经 McuDataPreprocessor 预处理）
    struct McuData {
        bool   valid = false;
        float  bullet_velocity = 0.0f;
        float  pitch_angle = 0.0f;          // 已标定（imu_euler_pitch 语义）
        double yaw_angle = 0.0;             // 多圈连续
        float  yaw_omega = 0.0f;
        float  chassis_imu_yaw = 0.0f;
        float  chassis_imu_omega = 0.0f;
        uint8_t mark = 0, color = 0, auto_aim_switch = 0, yaw_temperature = 0;
    };

    // IMU 原始数据
    struct ImuData {
        bool   valid = false;
        float  gx = 0.0f, gy = 0.0f, gz = 0.0f;
        float  ax = 0.0f, ay = 0.0f, az = 0.0f;
        double euler_yaw = 0.0, euler_pitch = 0.0, euler_roll = 0.0;
        uint32_t dt_one_tenth_ms = 0;
    };

    // FusionFilter 输出（YawChassisFusion::Output）
    struct FusedData {
        bool   valid = false;
        double yaw_pos = 0.0;           // yaw 关节解卷绕位置（多圈）
        double yaw_rate = 0.0;          // yaw 关节速度（高频）
        double chassis_yaw = 0.0;       // 底盘 world 系 yaw（解卷绕）
        double chassis_pitch = 0.0;
        double chassis_roll = 0.0;
        double imu_yaw_unwrapped = 0.0;
    };

    // 严格反解数据包（独立输出，非 FusedData 子包）：
    // 所有角度 wrap 到 (-π, π]；始终有效，缺失数据以 0 参与；
    // 保证 R_imu = R_chassis·Rz(yaw_pos)·Rx(pitch_angle) 恒成立
    struct StrictPose {
        double imu_euler_yaw = 0.0, imu_euler_pitch = 0.0, imu_euler_roll = 0.0;
        double yaw_pos = 0.0;       // 反解所用的 yaw_pos（wrap 后）
        double pitch_angle = 0.0;   // 反解所用的 pitch_angle（wrap 后）
        double chassis_yaw = 0.0;   // 严格反解结果（wrap 后）
        double chassis_pitch = 0.0;
        double chassis_roll = 0.0;
    };

    // MPC 状态
    struct MpcData {
        double yaw_target_angle = 0.0;      // 最新预测位置
        double yaw_target_velocity = 0.0;   // 最新预测速度
        double yaw_torque = 0.0;            // 最新控制力矩
        double delayed_target = 0.0;        // 当前参考（延迟 dt*N 步的目标）
        double integral = 0.0;              // 当前积分值（积分补偿累积量）
        std::vector<double> ref_sequence;   // 最新一次运算的目标位置序列（N 个）
        std::vector<double> pred_sequence;  // 最新一次运算的预测位置序列（N 个，不含当前）
    };

    // 统一状态：按来源分组
    struct State {
        McuData  mcu;
        ImuData  imu;
        FusedData fused;
        MpcData  mpc;
        StrictPose strict;   // 严格反解数据包（独立）
    };

    // 控制模式：单目标（默认）/ 序列
    // 构造时选定模式后，调用另一种模式的 set 接口会抛出 std::runtime_error
    enum class Mode { SINGLE = 0, SEQUENCE = 1 };

    // 建立通信 + MPC 控制封装（dt_control 控制周期、N 预测步数、MPC 参数；
    // integral_gain：yaw 力矩积分补偿比例系数（必须传参）；
    // mcu_linear_params：MCU 数据线性映射标定参数（默认构造为当前标定值）；
    // sequence_mode=true 选择序列模式）
    RobotController(double dt_control, int N,
                    double J, double tau_c, double b, double tau_d,
                    double max_torque, double max_torque_rate,
                    double Q, double R, double Rd, int max_iter,
                    double integral_gain,
                    const McuDataPreprocessor::LinearParams& mcu_linear_params,
                    bool sequence_mode = false);
    ~RobotController();

    // 统一获取：mcu/imu 原始 + 融合输出 + mpc 状态（含参考/预测序列与积分值）
    State getState();

    // 直通 McuMpcController::set（单目标模式；后台 100Hz 线程求解并发送 MCU）
    // integral_enable：本步是否启用 yaw 力矩积分补偿（必须传参；true 时
    //   yaw_torque 加积分后限幅，false 时积分清空为 0）。
    // 序列模式下调用本接口抛出 std::runtime_error
    void set(bool auto_aim_enable, bool yaw_torque_only_mode, double target_yaw,
             double pitch_target_angle, bool fire, bool integral_enable);

    // 序列模式 set（直通 McuMpcController 序列版 set）
    // integral_enable：本次序列执行期间是否启用 yaw 力矩积分补偿（必须传参）。
    // 单目标模式下调用本接口抛出 std::runtime_error
    void set(bool auto_aim_enable, bool yaw_torque_only_mode,
             const std::vector<double>& target_yaw_seq,
             const std::vector<double>& pitch_seq,
             const std::vector<bool>& fire_seq,
             bool integral_enable);

    Mode mode() const { return sequence_mode_ ? Mode::SEQUENCE : Mode::SINGLE; }

private:
    bool sequence_mode_ = false;
    RobotCommunication comm_;
    McuMpcController   mcu_mpc_;
};

#endif // ROBOT_CONTROLLER_H
