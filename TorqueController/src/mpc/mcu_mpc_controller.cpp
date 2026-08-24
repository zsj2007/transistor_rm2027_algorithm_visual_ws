#include "mpc/mcu_mpc_controller.h"

#include <chrono>
#include <cmath>
#include <algorithm>

McuMpcController::McuMpcController(RobotCommunication* comm, double dt_control, int N,
                                   double J, double tau_c, double b, double tau_d,
                                   double max_torque, double max_torque_rate,
                                   double Q, double R, double Rd, int max_iter,
                                   double integral_gain)
    : comm_(comm),
      mpc_(comm, dt_control, N,
           J, tau_c, b, tau_d,
           max_torque, max_torque_rate,
           Q, R, Rd, max_iter, integral_gain)
{}

McuMpcController::~McuMpcController() {
    stop();
}

void McuMpcController::start() {
    if (!running_.exchange(true)) {
        thread_ = std::thread(&McuMpcController::loop, this);
    }
}

void McuMpcController::stop() {
    if (running_.exchange(false)) {
        if (thread_.joinable()) thread_.join();
    }
}

void McuMpcController::set(bool auto_aim_enable, bool yaw_torque_only_mode,
                           double target_yaw, double pitch_target_angle, bool fire,
                           bool integral_enable) {
    // target_yaw 自动转换到与 imu_yaw_unwrapped 角度差最小的等效角：
    //   target_adj = imu_yaw + remainder(target_yaw − imu_yaw, 2π)
    //   - |target_adj − imu_yaw_unwrapped| ≤ π（角度差最小）
    //   - target_adj ≡ target_yaw (mod 2π)（与 target_yaw 同向/同角度）
    //   避免参考序列引入整圈偏差
    double imu_yaw = 0.0;
    if (comm_) {
        auto fused = comm_->getFused();
        if (fused.valid) imu_yaw = fused.imu_yaw_unwrapped;
    }
    double target_adj = imu_yaw + std::remainder(target_yaw - imu_yaw, 2.0 * M_PI);

    std::lock_guard<std::mutex> lock(set_mtx_);
    auto_aim_enable_      = auto_aim_enable;
    yaw_torque_only_mode_ = yaw_torque_only_mode;
    integral_enable_      = integral_enable;
    target_yaw_           = target_adj;
    pitch_target_angle_   = pitch_target_angle;
    fire_                 = fire;
    // 单目标 set：清空序列模式数据
    target_yaw_seq_.clear();
    pitch_seq_.clear();
    fire_seq_.clear();
}

// 序列版 set：不截断，三个序列各自独立存储；
// target_yaw 序列：第一个值 remainder 到 imu_yaw ±π 内，后续值 remainder 到前一个值 ±π 内
void McuMpcController::set(bool auto_aim_enable, bool yaw_torque_only_mode,
                           const std::vector<double>& target_yaw_seq,
                           const std::vector<double>& pitch_seq,
                           const std::vector<bool>& fire_seq,
                           bool integral_enable) {
    // imu_yaw（第一个 target 值的 wrap 基准）
    double imu_yaw = 0.0;
    if (comm_) {
        auto fused = comm_->getFused();
        if (fused.valid) imu_yaw = fused.imu_yaw_unwrapped;
    }

    std::lock_guard<std::mutex> lock(set_mtx_);
    auto_aim_enable_      = auto_aim_enable;
    yaw_torque_only_mode_ = yaw_torque_only_mode;
    integral_enable_      = integral_enable;

    target_yaw_seq_.clear();
    double prev = imu_yaw;
    for (double v : target_yaw_seq) {
        prev = prev + std::remainder(v - prev, 2.0 * M_PI);
        target_yaw_seq_.push_back(prev);
    }
    pitch_seq_.assign(pitch_seq.begin(), pitch_seq.end());
    fire_seq_.assign(fire_seq.begin(), fire_seq.end());
}

McuMpcController::State McuMpcController::state() const {
    std::lock_guard<std::mutex> lock(state_mtx_);
    return last_state_;
}

void McuMpcController::loop() {
    while (running_) {
        // 循环开始处获取本次循环时间基准
        auto start = std::chrono::steady_clock::now();

        // 取最新设置的发送参数与 mpc 目标；若序列模式非空则优先消费序列
        bool aa, mode, fire, integral_enable;
        double target_yaw, pitch;
        std::vector<double> seq_buf;   // 非空表示使用整序列 step
        {
            std::lock_guard<std::mutex> lock(set_mtx_);
            aa    = auto_aim_enable_;
            mode  = yaw_torque_only_mode_;
            integral_enable = integral_enable_;
            target_yaw = target_yaw_;
            pitch = pitch_target_angle_;
            fire  = fire_;

            // 各通道独立消费：序列非空时取首值覆盖成员并移除；
            // 某序列用完（空）时对应成员保持当前值（回原模式），其余通道继续用序列。
            // target_yaw 序列决定本次是否使用整序列 step。
            if (!target_yaw_seq_.empty()) {
                seq_buf.assign(target_yaw_seq_.begin(), target_yaw_seq_.end());
                target_yaw_ = target_yaw_seq_.front();
                target_yaw_seq_.pop_front();
                target_yaw = target_yaw_;
            }
            if (!pitch_seq_.empty()) {
                pitch_target_angle_ = pitch_seq_.front();
                pitch_seq_.pop_front();
                pitch = pitch_target_angle_;
            }
            if (!fire_seq_.empty()) {
                fire_ = fire_seq_.front();
                fire_seq_.pop_front();
                fire = fire_;
            }
        }

        // mpc 求解（序列模式传整个目标缓冲；内部读融合状态）
        auto res = seq_buf.empty() ? mpc_.step(target_yaw, integral_enable)
                                   : mpc_.step(seq_buf, integral_enable);

        // 配合最新设置构造发送包
        mcu::SendPacket pkt;
        pkt.auto_aim_enable    = aa ? 1 : 0;
        pkt.fire               = fire ? 1 : 0;
        pkt.pitch_target_angle = static_cast<float>(pitch);
        pkt.yaw_torque_only_mode = mode ? 1 : 0;
        pkt.yaw_target_angle   = res.yaw_target_angle;
        pkt.yaw_target_velocity = static_cast<float>(res.yaw_target_velocity);
        pkt.yaw_torque         = static_cast<float>(res.yaw_torque);
        if (comm_) comm_->sendToMcu(pkt);

        // 缓存最新结果（供显示）
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            last_state_.yaw_target_angle    = res.yaw_target_angle;
            last_state_.yaw_target_velocity = res.yaw_target_velocity;
            last_state_.yaw_torque          = res.yaw_torque;
            last_state_.delayed_target      = res.delayed_target;
            last_state_.integral            = mpc_.integral();
            last_state_.ref_sequence        = res.ref_sequence;
            last_state_.pred_sequence       = res.pred_sequence;
        }

        // 循环结束处：等待到 start + 10ms（100Hz），不严格跟随绝对时间点
        std::this_thread::sleep_until(start + std::chrono::milliseconds(10));
    }
}
