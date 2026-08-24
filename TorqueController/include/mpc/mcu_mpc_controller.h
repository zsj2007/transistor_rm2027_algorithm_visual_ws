#ifndef MCU_MPC_CONTROLLER_H
#define MCU_MPC_CONTROLLER_H

#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>
#include "communication/Communications.hpp"
#include "mpc/yaw_mpc_controller.h"

// ============================================================================
// McuMpcController — 实车 MCU 控制封装（yaw MPC + 发送参数维护 + 后台发送线程）
//
// - set(auto_aim_enable, yaw_torque_only_mode, target_yaw, pitch_target_angle,
//      fire): 一次性设置全部发送参数与 mpc 目标。除 target_yaw 传给
//      YawMpcController 外，其余参数由本类自己维护（后台线程读取使用）。
//      target_yaw 在设置时自动转换到与 imu_yaw_unwrapped 角度差最小的等效角
//      （|差| ≤ π，且与 target_yaw 同向：target_adj ≡ target_yaw (mod 2π)）。
// - 后台线程固定 100Hz：从本类取最新 target_yaw 调用 YawMpcController::step，
//   以 mpc 结果配合最新设置的发送参数构造 McuSendPacket 发送给 MCU。
// - 线程循环：开始处取 steady_clock::now()，结束处
//   sleep_until(start + 10ms)，不严格跟随绝对时间点，避免误差累计。
// - 设置参数与最新结果（last_state_）使用独立的锁保护。
// ============================================================================
class McuMpcController {
public:
    // 最新 mpc 求解结果（供显示/日志）
    struct State {
        double yaw_target_angle = 0.0;
        double yaw_target_velocity = 0.0;
        double yaw_torque = 0.0;
        double delayed_target = 0.0;
        double integral = 0.0;      // 当前积分值（积分补偿累积量）
        std::vector<double> ref_sequence;   // 本次参考（目标）序列（N 个）
        std::vector<double> pred_sequence;  // 本次预测位置序列（N 个）
    };

    McuMpcController(RobotCommunication* comm, double dt_control, int N,
                     double J, double tau_c, double b, double tau_d,
                     double max_torque, double max_torque_rate,
                     double Q, double R, double Rd, int max_iter,
                     double integral_gain = 0.01);
    ~McuMpcController();

    void start();   // 启动后台 100Hz 发送线程（可重复调用，幂等）
    void stop();    // 停止并 join

    // 设置发送参数 + mpc 目标（线程安全）。
    // target_yaw 自动转换到与 imu_yaw_unwrapped 角度差最小的等效角（与 target 同向）。
    // integral_enable: 本步是否启用 yaw 力矩积分补偿（透传 YawMpcController::step；
    //   true 时 yaw_torque 加积分后限幅，false 时积分清空为 0）。
    void set(bool auto_aim_enable, bool yaw_torque_only_mode, double target_yaw,
             double pitch_target_angle, bool fire, bool integral_enable = false);

    // 序列版 set：传入 target_yaw / pitch_target_angle / fire 三个序列
    // （不截断，各序列独立存储；target_yaw 序列：第一个值 remainder 到
    //  imu_yaw ±π 内，后续值 remainder 到前一个值 ±π 内）。存储到内部序列成员，
    //  后台线程按序逐通道消费：某序列非空取首值，空则对应成员保持当前值（回原模式），
    //  其余通道继续用序列。调用单目标 set 会清空这些序列。
    // integral_enable: 本次序列执行期间是否启用 yaw 力矩积分补偿（透传 step）。
    void set(bool auto_aim_enable, bool yaw_torque_only_mode,
             const std::vector<double>& target_yaw_seq,
             const std::vector<double>& pitch_seq,
             const std::vector<bool>& fire_seq,
             bool integral_enable = false);

    // 最新 mpc 结果（线程安全，显示用）
    State state() const;

private:
    void loop();

    RobotCommunication* comm_;
    YawMpcController mpc_;

    // 设置参数锁（后台线程读取）
    mutable std::mutex set_mtx_;
    bool   auto_aim_enable_ = true;
    bool   yaw_torque_only_mode_ = false;
    bool   integral_enable_ = false;   // 积分补偿开关（后台线程读取）
    double target_yaw_ = 0.0;
    double pitch_target_angle_ = 0.0;
    bool   fire_ = false;

    // 序列模式成员（非空时 loop 优先消费）
    std::deque<double> target_yaw_seq_;
    std::deque<double> pitch_seq_;
    std::deque<bool>   fire_seq_;

    // 最新结果锁（显示线程读取）
    mutable std::mutex state_mtx_;
    State  last_state_;

    std::thread thread_;
    std::atomic<bool> running_{false};
};

#endif // MCU_MPC_CONTROLLER_H
