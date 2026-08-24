// pitch_calibration.cpp
// Pitch 轴标定程序
// 采样 target_angle(pitch_target_angle), imu_euler_pitch, mcu_pitch_angle 的关系，进行两级线性拟合
//   Fit1: mcu_pitch_angle → imu_euler_pitch
//   Fit2: imu_euler_pitch → pitch_target_angle
//
// 使用 0.1 和 0.9 分位值而非直接使用 min/max 的原因：
// 物理系统在行程端点附近通常存在非线性（如机械限位、电机力矩饱和、
// 传感器边缘效应等），端点处的测量值噪声也更大。取 0.1~0.9 分位范围
// 可以剔除两端各 10% 的不可靠数据，聚焦在线性度最好的中间区域进行
// 拟合，得到的斜率和截距更能代表系统的真实线性特性，避免端点异常值
// 拉偏回归结果。

#include "communication/Communications.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <thread>
#include <csignal>

namespace {

void signalHandler(int) { 
    std::cout << "\n用户中断。\n";
    exit(0);
}

struct DataPoint {
    float target_angle;  // mcu::SendPacket.pitch_target_angle
    float imu_pitch;     // imu::ReceivePacket.euler_pitch
    float mcu_pitch;     // mcu::ReceivePacket.pitch_angle
};

struct LinearFit {
    float slope     = 0;
    float intercept = 0;
    float r_squared = 0;
};

class PitchCalibrator {
public:
    PitchCalibrator(float target_min, float target_max, int fit_points = 20);
    ~PitchCalibrator();
    void run();

private:
    McuCommunication serial_;
    ImuCommunication imu_serial_;
    std::mutex              data_mutex_;
    mcu::ReceivePacket      latest_packet_{};
    bool                    has_data_ = false;

    std::mutex              imu_mutex_;
    imu::ReceivePacket      latest_imu_packet_{};
    bool                    has_imu_data_ = false;

    float target_min_, target_max_;
    int   fit_points_;

    void onPacket(const mcu::ReceivePacket& pkt);
    void onImuPacket(const imu::ReceivePacket& pkt);
    DataPoint sample(float target_angle);
    float binarySearchTarget(float target_y, float lo, float hi,
                              float sign, int max_iter = 8);
    static LinearFit fitLinear(const std::vector<float>& xs,
                                const std::vector<float>& ys);
    static float lerp(float a, float b, float t) { return a + t * (b - a); }
};

PitchCalibrator::PitchCalibrator(float target_min, float target_max, int fit_points)
    : serial_([this](const mcu::ReceivePacket& pkt) { onPacket(pkt); })
    , imu_serial_([this](const imu::ReceivePacket& pkt) { onImuPacket(pkt); })
    , target_min_(target_min), target_max_(target_max), fit_points_(fit_points)
{}

PitchCalibrator::~PitchCalibrator() { serial_.stopWorker(); imu_serial_.stopWorker(); }

void PitchCalibrator::onPacket(const mcu::ReceivePacket& pkt) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_packet_ = pkt;
    has_data_ = true;
}

void PitchCalibrator::onImuPacket(const imu::ReceivePacket& pkt) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_packet_ = pkt;
    has_imu_data_ = true;
}

DataPoint PitchCalibrator::sample(float target_angle) {
    mcu::SendPacket pkt;
    pkt.auto_aim_enable    = 1;
    pkt.fire               = 0;
    pkt.pitch_target_angle = target_angle;
    pkt.yaw_torque_only_mode = 1;
    pkt.yaw_target_angle   = 0.0;
    pkt.yaw_target_velocity = 0.0f;
    pkt.yaw_torque         = 0.0f;
    serial_.sendData(pkt);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    DataPoint dp{};
    dp.target_angle = target_angle;

    // 同时采集 MCU 和 IMU 数据
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (has_data_) dp.mcu_pitch = latest_packet_.pitch_angle;
    }
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (has_imu_data_)
            dp.imu_pitch = static_cast<float>(latest_imu_packet_.euler_pitch);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return dp;
}

float PitchCalibrator::binarySearchTarget(float target_y,
                                           float lo, float hi,
                                           float sign, int max_iter) {
    for (int i = 0; i < max_iter; ++i) {
        float mid = (lo + hi) * 0.5f;
        float y   = sample(mid).mcu_pitch;
        std::cout << "    [" << i + 1 << "/" << max_iter << "] target="
                  << std::fixed << std::setprecision(4) << mid
                  << " -> mcu=" << y << " (target_y=" << target_y << ")\n";
        if (sign * (y - target_y) < 0) lo = mid;
        else                           hi = mid;
    }
    return (lo + hi) * 0.5f;
}

LinearFit PitchCalibrator::fitLinear(const std::vector<float>& xs,
                                      const std::vector<float>& ys) {
    LinearFit result;
    size_t n = xs.size();
    if (n < 2) return result;
    float sx = 0, sy = 0, sxy = 0, sx2 = 0, sy2 = 0;
    for (size_t i = 0; i < n; ++i) {
        float x = xs[i], y = ys[i];
        sx  += x;  sy  += y;
        sxy += x * y;
        sx2 += x * x;
        sy2 += y * y;
    }
    float denom = n * sx2 - sx * sx;
    if (std::fabs(denom) < 1e-9f) return result;
    result.slope     = (n * sxy - sx * sy) / denom;
    result.intercept = (sy - result.slope * sx) / n;
    float my = sy / n, ssr = 0, sst = 0;
    for (size_t i = 0; i < n; ++i) {
        float pred = result.slope * xs[i] + result.intercept;
        ssr += (ys[i] - pred) * (ys[i] - pred);
        sst += (ys[i] - my)   * (ys[i] - my);
    }
    result.r_squared = (sst > 1e-9f) ? 1.0f - ssr / sst : 1.0f;
    return result;
}

void PitchCalibrator::run() {
    // ── Step 1: 测量两端点 ──
    std::cout << "\n========== Step 1: 测量两端点 ==========\n";
    std::cout << "target_min = " << target_min_ << ", target_max = " << target_max_ << "\n";

    float y_left  = sample(target_min_).mcu_pitch;
    float y_right = sample(target_max_).mcu_pitch;

    std::cout << "y_left  = " << y_left  << "  (at target=" << target_min_ << ")\n";
    std::cout << "y_right = " << y_right << "  (at target=" << target_max_ << ")\n";

    float sign = (y_right > y_left) ? 1.0f : -1.0f;
    std::cout << "相关性: " << (sign > 0 ? "正相关" : "负相关") << "\n";

    // ── Step 2: 二分查找 y 中心值对应的 target_angle ──
    float y_mid = (y_left + y_right) * 0.5f;
    std::cout << "\n========== Step 2: 二分查找 y 中心 ==========\n";
    std::cout << "y_mid = " << y_mid << "\n";

    float target_center = binarySearchTarget(y_mid, target_min_, target_max_, sign, 8);
    std::cout << "target_center = " << target_center << "\n";

    // ── Step 3: 二分查找 0.1 / 0.9 分位值对应的 target_angle ──
    float y_min = std::min(y_left, y_right);
    float y_max = std::max(y_left, y_right);
    float y_01 = lerp(y_min, y_max, 0.1f);
    float y_09 = lerp(y_min, y_max, 0.9f);

    std::cout << "\n========== Step 3: 查找 0.1 / 0.9 分位 ==========\n";
    std::cout << "y 范围: [" << y_min << ", " << y_max << "]\n";
    std::cout << "y_0.1 = " << y_01 << ", y_0.9 = " << y_09 << "\n";

    float target_01, target_09;
    if (y_left < y_right) {
        std::cout << "\n--- 向 target_min 方向搜索 target_0.1 ---\n";
        target_01 = binarySearchTarget(y_01, target_min_, target_center, sign, 8);
        std::cout << "--- 向 target_max 方向搜索 target_0.9 ---\n";
        target_09 = binarySearchTarget(y_09, target_center, target_max_, sign, 8);
    } else {
        std::cout << "\n--- 向 target_max 方向搜索 target_0.1 ---\n";
        target_01 = binarySearchTarget(y_01, target_center, target_max_, sign, 8);
        std::cout << "--- 向 target_min 方向搜索 target_0.9 ---\n";
        target_09 = binarySearchTarget(y_09, target_min_, target_center, sign, 8);
    }

    std::cout << "target_0.1 = " << target_01 << ", target_0.9 = " << target_09 << "\n";

    // ── Step 4: 在 [target_01, target_09] 范围内交替采样 ──
    float target_lo = std::min(target_01, target_09);
    float target_hi = std::max(target_01, target_09);

    std::cout << "\n========== Step 4: 交替采样拟合数据 ==========\n";
    std::cout << "采样范围: [" << target_lo << ", " << target_hi << "]\n";

    // 构建交替测量顺序：从两端向中间交替取值，避免单向漂移引入系统误差
    std::vector<float> target_order;
    target_order.reserve(fit_points_);
    {
        int lo = 0, hi = fit_points_ - 1;
        while (lo <= hi) {
            float t_lo = float(lo) / std::max(1, fit_points_ - 1);
            target_order.push_back(lerp(target_lo, target_hi, t_lo));
            lo++;
            if (lo > hi) break;
            float t_hi = float(hi) / std::max(1, fit_points_ - 1);
            target_order.push_back(lerp(target_lo, target_hi, t_hi));
            hi--;
        }
    }

    std::vector<DataPoint> samples;
    for (int i = 0; i < fit_points_; ++i) {
        DataPoint dp = sample(target_order[i]);
        samples.push_back(dp);
        std::cout << "  [" << std::setw(2) << i + 1 << "/" << fit_points_ << "]"
                  << " target=" << std::fixed << std::setprecision(4) << dp.target_angle
                  << " -> imu=" << dp.imu_pitch << " mcu=" << dp.mcu_pitch << "\n";
    }

    // ── Step 5: 两级线性拟合 ──
    std::cout << "\n========== Step 5: 线性拟合结果 ==========\n";

    // 提取用于拟合的向量
    std::vector<float> target_vec, imu_vec, mcu_vec;
    target_vec.reserve(samples.size());
    imu_vec.reserve(samples.size());
    mcu_vec.reserve(samples.size());
    for (auto& dp : samples) {
        target_vec.push_back(dp.target_angle);
        imu_vec.push_back(dp.imu_pitch);
        mcu_vec.push_back(dp.mcu_pitch);
    }

    // Fit1: mcu_pitch_angle → imu_euler_pitch
    LinearFit fit1 = fitLinear(mcu_vec, imu_vec);

    std::cout << "\n--- Fit1: mcu_pitch_angle → imu_euler_pitch ---\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "斜率 (slope)      : " << fit1.slope << "\n";
    std::cout << "截距 (intercept)  : " << fit1.intercept << "\n";
    std::cout << "R²                : " << fit1.r_squared << "\n";
    std::cout << "拟合公式: imu_euler_pitch = " << fit1.slope
              << " * mcu_pitch_angle + " << fit1.intercept << "\n";

    // Fit2: imu_euler_pitch → pitch_target_angle
    LinearFit fit2 = fitLinear(imu_vec, target_vec);

    std::cout << "\n--- Fit2: imu_euler_pitch → pitch_target_angle ---\n";
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "斜率 (slope)      : " << fit2.slope << "\n";
    std::cout << "截距 (intercept)  : " << fit2.intercept << "\n";
    std::cout << "R²                : " << fit2.r_squared << "\n";
    std::cout << "拟合公式: pitch_target_angle = " << fit2.slope
              << " * imu_euler_pitch + " << fit2.intercept << "\n";

    // 组合公式: 由 mcu_pitch_angle 直接推算 pitch_target_angle
    std::cout << "\n--- 组合公式: mcu_pitch_angle → pitch_target_angle ---\n";
    float comb_slope     = fit2.slope * fit1.slope;
    float comb_intercept = fit2.slope * fit1.intercept + fit2.intercept;
    std::cout << "pitch_target_angle = " << comb_slope << " * mcu_pitch_angle + "
              << comb_intercept << "\n";

    // y 极值（MCU pitch_angle）取自 Step 3 已计算的 y_min / y_max
    float target_at_y_min = (std::fabs(comb_slope) > 1e-9f)
                            ? comb_slope * y_min + comb_intercept : target_min_;
    float target_at_y_max = (std::fabs(comb_slope) > 1e-9f)
                            ? comb_slope * y_max + comb_intercept : target_max_;

    std::cout << std::setprecision(4);
    std::cout << "\nMCU pitch_angle 最小值 : " << y_min
              << "  (pitch_target_angle=" << target_at_y_min << ")\n";
    std::cout << "MCU pitch_angle 最大值 : " << y_max
              << "  (pitch_target_angle=" << target_at_y_max << ")\n";

    // ── Step 6: 输出可直接替换 LinearParams 默认值部分的片段（不自动写回代码）──
    std::cout << "\n========== Step 6: LinearParams 默认值替换片段 ==========\n";
    std::cout << "以下 4 行可整体替换 include/communication/McuDataPreprocessor.h 中\n";
    std::cout << "struct LinearParams 的默认值部分（缩进与注释与当前代码一致）：\n\n";

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "        double send_pitch_scale  = " << std::setw(10) << fit2.slope
              << ";   // imu_euler_pitch → pitch_target_angle\n";
    std::cout << "        double send_pitch_offset = " << std::setw(10) << fit2.intercept << ";\n";
    std::cout << "        double recv_pitch_scale  = " << std::setw(10) << fit1.slope
              << ";   // mcu_pitch_angle → imu_euler_pitch\n";
    std::cout << "        double recv_pitch_offset = " << std::setw(10) << fit1.intercept << ";\n";

    std::cout << "\n========================================\n";
    std::cout << "标定完成。\n";
}

} // namespace

int main() {
    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    // ─── 硬编码参数（按需修改） ───
    constexpr float target_min =  -10.0f;
    constexpr float target_max =  30.0f;
    constexpr int   n_pts      =  20;

    std::cout << "Pitch 轴标定程序\n";
    std::cout << "target_angle 范围: [" << target_min << ", " << target_max << "]\n";
    std::cout << "拟合采样点数: " << n_pts << "\n";
    std::cout << "按 Ctrl+C 可随时中断\n";

    PitchCalibrator calib(target_min, target_max, n_pts);
    calib.run();

    return 0;
}
