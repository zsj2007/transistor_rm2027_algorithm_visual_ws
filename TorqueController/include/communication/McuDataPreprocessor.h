#pragma once

#include "Protocol.hpp"
#include <cmath>

// ============================================================================
// McuDataPreprocessor — MCU 通信数据预处理类
//
// 线性映射参数在构造时传入（默认参数为当前标定值），
// 便于通过标定工具（如 pitch_calibration）运行时配置。
// ============================================================================
class McuDataPreprocessor {
public:

    // ── 线性映射参数（默认值为当前标定值）──
    struct LinearParams {
        double send_pitch_scale  =  20.523245;   // imu_euler_pitch → pitch_target_angle
        double send_pitch_offset =   0.475049;
        double recv_pitch_scale  =   1.122635;   // mcu_pitch_angle → imu_euler_pitch
        double recv_pitch_offset =  -0.170755;
    };

    // 构造时传入线性映射参数（默认使用当前标定值）
    explicit McuDataPreprocessor(const LinearParams& params = defaultParams())
        : params_(params) {}

    // 当前标定默认值（LinearParams 的默认成员初始化器）
    static LinearParams defaultParams() { return LinearParams{}; }

    // ── 发送包预处理 ──
    mcu::SendPacket processSend(const mcu::SendPacket& packet);

    // ── 接收包预处理 ──
    mcu::ReceivePacket processReceive(const mcu::ReceivePacket& packet);

private:
    LinearParams params_;
};
