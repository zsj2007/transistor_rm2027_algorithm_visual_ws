// control_demo.cpp — 使用 RobotController 封装类的正弦目标演示程序
//
// - 参数与 pygame_control_mpc.py 一致
// - 以 3s 周期正弦控制 target_yaw（±30°）与 pitch_target_angle（-10°~+20°），
//   两者相位差 90°
// - 内部：RobotController 建立通信 + 融合 + yaw MPC + 后台 100Hz 发送线程，
//   外部只需 set() 设置目标即可
// - 运行: ./build/control_demo，Ctrl+C 退出
#include "RobotController.h"

#include <cmath>
#include <cstdio>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

namespace {

std::atomic<bool> g_running{true};
void signalHandler(int) { g_running = false; }

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double PERIOD_S = 3.0;              // 正弦周期（秒）
constexpr double OMEGA = 2.0 * M_PI / PERIOD_S;

// ── 参数（与 pygame_control_mpc.py 一致）──
constexpr double DT_CTRL = 0.01;              // 控制周期 (100 Hz)
constexpr int    MPC_PRED_N = 20;             // int(DELAY_TIME / DT_CTRL)，DELAY_TIME=0.2s
constexpr double J      = 0.016541;
constexpr double TAU_C  = 0.097297;
constexpr double B_FRIC = 0.032100;
constexpr double TAU_D  = 0.0;
constexpr double MAX_TORQUE      = 1.0;
constexpr double MAX_TORQUE_RATE = 40.0;
constexpr double Q = 5.0;
constexpr double R = 0.01;
constexpr double Rd = 0.1;
constexpr int    MAX_ITER = 30;

// ── 正弦目标 ──
// target_yaw: ±30° 正弦
inline double targetYaw(double t) {
    return 30.0 * DEG2RAD * std::sin(OMEGA * t);
}

// pitch_target_angle: 范围 -10°~+20°（中点 5°、幅值 15°），与 yaw 相位差 90°
inline double targetPitch(double t) {
    return (5.0 + 15.0 * std::sin(OMEGA * t - M_PI / 2.0)) * DEG2RAD;
}

} // namespace

int main() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    printf("=== control_demo (RobotController) ===\n");
    printf("正弦周期 %.1fs: target_yaw ±30°, pitch_target -10°~+20°, 相位差 90°\n", PERIOD_S);

    // 一体化控制封装：通信 + 融合滤波器 + yaw MPC + 后台 100Hz 发送线程
    // integral_gain: yaw 力矩积分补偿比例系数（默认 0.01）
    // mcu_linear_params 使用当前默认标定值（LinearParams{} 与默认构造等价）
    RobotController rc(DT_CTRL, MPC_PRED_N,
                       J, TAU_C, B_FRIC, TAU_D,
                       MAX_TORQUE, MAX_TORQUE_RATE,
                       Q, R, Rd, MAX_ITER,
                       /*integral_gain=*/0.01,
                       McuDataPreprocessor::LinearParams{});

    // 等待融合数据就绪
    printf("等待融合数据就绪...\n");
    while (g_running) {
        if (rc.getState().fused.valid) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!g_running) return 0;
    printf("融合就绪，开始正弦控制（Ctrl+C 退出）\n");

    auto t0 = std::chrono::steady_clock::now();
    int loop = 0;
    while (g_running) {
        auto start = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(start - t0).count();

        // 设置发送参数 + mpc 目标（后台线程 100Hz 求解并发送）
        double yaw   = targetYaw(t);
        double pitch = targetPitch(t);
        rc.set(/*auto_aim_enable=*/true, /*yaw_torque_only_mode=*/false,
               yaw, pitch, /*fire=*/false, /*integral_enable=*/false);

        // 每 0.1s 打印一次
        if (++loop % 10 == 0) {
            auto st = rc.getState();
            printf("[t=%6.2fs] target_yaw=%+7.2f° pitch=%+6.2f° | "
                   "yaw_pos=%+.3f yaw_rate=%+.3f torque=%+.4f | fused=%d | temperature=%d\n",
                   t, yaw / DEG2RAD, pitch / DEG2RAD,
                   st.fused.yaw_pos, st.fused.yaw_rate, st.mpc.yaw_torque,
                   (int)st.fused.valid, st.mcu.yaw_temperature);
        }

        // 循环结束处等待到 start + 10ms（100Hz），不累计误差
        std::this_thread::sleep_until(start + std::chrono::milliseconds(10));
    }

    printf("\n退出。\n");
    return 0;
}
