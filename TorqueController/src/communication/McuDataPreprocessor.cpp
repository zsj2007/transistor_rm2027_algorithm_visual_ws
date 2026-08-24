#include "communication/McuDataPreprocessor.h"
#include <cmath>

mcu::SendPacket McuDataPreprocessor::processSend(const mcu::SendPacket& packet) {
    mcu::SendPacket result = packet;
    result.pitch_target_angle = params_.send_pitch_scale * packet.pitch_target_angle + params_.send_pitch_offset; // imu_euler_pitch → pitch_target_angle
    return result;
}

mcu::ReceivePacket McuDataPreprocessor::processReceive(const mcu::ReceivePacket& packet) {
    // static constexpr float YAW_SCALE = 2.0f * M_PI / 8192.0f;

    mcu::ReceivePacket result = packet;
    result.pitch_angle = params_.recv_pitch_scale * packet.pitch_angle + params_.recv_pitch_offset;             // mcu_pitch_angle → imu_euler_pitch
    result.yaw_angle   = packet.yaw_angle;// * YAW_SCALE;                              // 编码器值 → 弧度
    return result;
}
