// RobotCommunicationC.cpp — C API 实现，通过 memcpy 在 C/C++ 类型间转换

#include "c_api/RobotCommunicationC.h"
#include "communication/Communications.hpp"
#include "mpc/yaw_mpc_controller.h"
#include "mpc/mcu_mpc_controller.h"
#include <cstring>
#include <new>

// ============================================================================
// 编译期验证：C 结构体与 C++ 结构体逐字段偏移一致
// ============================================================================
#define ASSERT_OFFSET(CType, CppType, field) \
    static_assert(offsetof(CType, field) == offsetof(CppType, field), "offset mismatch: " #field)

ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, frame_header1);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, frame_header2);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, protocol_version);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, data_size);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, auto_aim_enable);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, fire);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, pitch_target_angle);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, yaw_torque_only_mode);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, yaw_target_angle);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, yaw_target_velocity);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, yaw_torque);
ASSERT_OFFSET(McuSendPacket_C, mcu::SendPacket, crc8);

ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, frame_header1);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, frame_header2);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, protocol_version);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, data_size);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, bullet_velocity);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, pitch_angle);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, yaw_angle);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, yaw_omega);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, chassis_imu_yaw);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, chassis_imu_omega);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, mark);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, color);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, auto_aim_switch);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, yaw_temperature);
ASSERT_OFFSET(McuReceivePacket_C, mcu::ReceivePacket, crc8);

static_assert(sizeof(McuSendPacket_C)    == sizeof(mcu::SendPacket),    "McuSendPacket size mismatch");
static_assert(sizeof(McuReceivePacket_C) == sizeof(mcu::ReceivePacket), "McuReceivePacket size mismatch");
static_assert(sizeof(ImuSendPacket_C)    == sizeof(imu::SendPacket),    "ImuSendPacket size mismatch");
static_assert(sizeof(ImuReceivePacket_C) == sizeof(imu::ReceivePacket), "ImuReceivePacket size mismatch");

// ============================================================================
// 内部实现：持有 RobotCommunication 实例
// ============================================================================
struct RobotCommHandle {
    RobotCommunication impl;
};

// ============================================================================
// 辅助转换函数
// ============================================================================
static inline void copyToCpp(const McuSendPacket_C& src, mcu::SendPacket& dst) {
    std::memcpy(&dst, &src, sizeof(src));
}
static inline void copyToC(const mcu::ReceivePacket& src, McuReceivePacket_C& dst) {
    std::memcpy(&dst, &src, sizeof(src));
}
static inline void copyToCpp(const ImuSendPacket_C& src, imu::SendPacket& dst) {
    std::memcpy(&dst, &src, sizeof(src));
}
static inline void copyToC(const imu::ReceivePacket& src, ImuReceivePacket_C& dst) {
    std::memcpy(&dst, &src, sizeof(src));
}

// ============================================================================
// API 实现
// ============================================================================

extern "C" {

RobotCommHandle* robot_comm_create(void) {
    return new (std::nothrow) RobotCommHandle;
}

void robot_comm_destroy(RobotCommHandle* handle) {
    delete handle;
}

RobotLatestData_C robot_comm_get_latest_data(RobotCommHandle* handle) {
    RobotLatestData_C result{};
    if (!handle) return result;

    auto data = handle->impl.getLatestData();

    result.imu_valid = data.imu_valid;
    result.mcu_valid = data.mcu_valid;

    if (data.imu_valid)
        copyToC(data.imu_packet, result.imu_packet);
    if (data.mcu_valid)
        copyToC(data.mcu_packet, result.mcu_packet);

    return result;
}

RobotFusedData_C robot_comm_get_fused_data(RobotCommHandle* handle) {
    RobotFusedData_C result{};
    if (!handle) return result;

    auto f = handle->impl.getFused();

    result.valid             = f.valid;
    result.yaw_pos           = f.yaw_pos;
    result.yaw_rate          = f.yaw_rate;
    result.chassis_yaw       = f.chassis_yaw;
    result.chassis_pitch     = f.chassis_pitch;
    result.chassis_roll      = f.chassis_roll;
    result.imu_yaw_unwrapped = f.imu_yaw_unwrapped;

    return result;
}

RobotStrictPose_C robot_comm_get_strict_pose(RobotCommHandle* handle) {
    RobotStrictPose_C result{};
    if (!handle) return result;

    auto sp = handle->impl.getStrictPose();

    result.imu_euler_yaw    = sp.imu_euler_yaw;
    result.imu_euler_pitch  = sp.imu_euler_pitch;
    result.imu_euler_roll   = sp.imu_euler_roll;
    result.yaw_pos          = sp.yaw_pos;
    result.pitch_angle      = sp.pitch_angle;
    result.chassis_yaw      = sp.chassis_yaw;
    result.chassis_pitch    = sp.chassis_pitch;
    result.chassis_roll     = sp.chassis_roll;

    return result;
}

bool robot_comm_send_to_mcu(RobotCommHandle* handle, const McuSendPacket_C* packet) {
    if (!handle || !packet) return false;

    mcu::SendPacket cpp_pkt;
    copyToCpp(*packet, cpp_pkt);

    return handle->impl.sendToMcu(cpp_pkt);
}

bool robot_comm_send_to_imu(RobotCommHandle* handle, const ImuSendPacket_C* packet) {
    if (!handle || !packet) return false;

    imu::SendPacket cpp_pkt;
    copyToCpp(*packet, cpp_pkt);
    return handle->impl.sendToImu(cpp_pkt);
}

void robot_comm_stop(RobotCommHandle* handle) {
    if (handle)
        handle->impl.stop();
}

// ============================================================================
// 实车 MPC 控制器 C API（替代旧 mpc_c_api）
// ============================================================================

YawMpcHandle_C yaw_mpc_create(RobotCommHandle* comm, double dt_control, int N,
                              double J, double tau_c, double b, double tau_d,
                              double max_torque, double max_torque_rate,
                              double Q, double R, double Rd, int max_iter) {
    if (!comm) return nullptr;
    try {
        return static_cast<YawMpcHandle_C>(new YawMpcController(
            &comm->impl, dt_control, N,
            J, tau_c, b, tau_d,
            max_torque, max_torque_rate,
            Q, R, Rd, max_iter));
    } catch (...) {
        return nullptr;
    }
}

void yaw_mpc_destroy(YawMpcHandle_C handle) {
    delete static_cast<YawMpcController*>(handle);
}

YawMpcStepResult_C yaw_mpc_step(YawMpcHandle_C handle, double target_yaw) {
    YawMpcStepResult_C out{};
    if (!handle) return out;
    auto r = static_cast<YawMpcController*>(handle)->step(target_yaw);
    out.yaw_target_angle    = r.yaw_target_angle;
    out.yaw_target_velocity = r.yaw_target_velocity;
    out.yaw_torque          = r.yaw_torque;
    out.delayed_target      = r.delayed_target;
    return out;
}

// ============================================================================
// 实车 MCU 控制封装 C API（设置 + 后台 100Hz 发送线程）
// ============================================================================

McuMpcHandle_C mcu_mpc_create(RobotCommHandle* comm, double dt_control, int N,
                              double J, double tau_c, double b, double tau_d,
                              double max_torque, double max_torque_rate,
                              double Q, double R, double Rd, int max_iter) {
    if (!comm) return nullptr;
    try {
        auto* ctl = new McuMpcController(
            &comm->impl, dt_control, N,
            J, tau_c, b, tau_d,
            max_torque, max_torque_rate,
            Q, R, Rd, max_iter);
        ctl->start();   // 自动启动后台 100Hz 发送线程
        return static_cast<McuMpcHandle_C>(ctl);
    } catch (...) {
        return nullptr;
    }
}

void mcu_mpc_destroy(McuMpcHandle_C handle) {
    delete static_cast<McuMpcController*>(handle);   // 析构内 stop + join
}

void mcu_mpc_step(McuMpcHandle_C handle,
                  uint8_t auto_aim_enable, uint8_t yaw_torque_only_mode,
                  double target_yaw, float pitch_target_angle, uint8_t fire) {
    if (!handle) return;
    static_cast<McuMpcController*>(handle)->set(
        auto_aim_enable != 0, yaw_torque_only_mode != 0, target_yaw,
        pitch_target_angle, fire != 0);
}

McuMpcState_C mcu_mpc_get_state(McuMpcHandle_C handle) {
    McuMpcState_C out{};
    if (!handle) return out;
    auto s = static_cast<McuMpcController*>(handle)->state();
    out.yaw_target_angle    = s.yaw_target_angle;
    out.yaw_target_velocity = s.yaw_target_velocity;
    out.yaw_torque          = s.yaw_torque;
    out.delayed_target      = s.delayed_target;
    return out;
}

} // extern "C"
