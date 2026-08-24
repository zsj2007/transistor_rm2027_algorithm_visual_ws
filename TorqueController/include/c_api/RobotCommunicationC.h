// RobotCommunicationC.h — RobotCommunication 的 C 语言 API
//
// 使用方式（Python ctypes / C 程序）:
//   1. robot_comm_create() 创建句柄
//   2. robot_comm_get_latest_data() 获取最新 IMU + MCU 数据
//   3. robot_comm_send_to_mcu / send_to_imu 发送数据
//   4. robot_comm_destroy() 销毁
//
// 注意：所有结构体为 #pragma pack(1)，与 Protocol.hpp 内存布局严格一致

#ifndef ROBOT_COMMUNICATION_C_H
#define ROBOT_COMMUNICATION_C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// C 兼容的数据包结构体（与 Protocol.hpp 中的 C++ 版本内存布局一致）
// ============================================================================

#pragma pack(push, 1)

// ── MCU 发送包 ──
typedef struct {
    uint8_t frame_header1;       // = 0x42
    uint8_t frame_header2;       // = 0x52
    uint8_t protocol_version;    // = 0x02
    uint8_t data_size;           // = 23
    uint8_t auto_aim_enable;
    uint8_t fire;
    float   pitch_target_angle;
    uint8_t yaw_torque_only_mode;
    double  yaw_target_angle;
    float   yaw_target_velocity;
    float   yaw_torque;
    uint8_t crc8;
} McuSendPacket_C;

// ── MCU 接收包 ──
typedef struct {
    uint8_t frame_header1;       // = 0x42
    uint8_t frame_header2;       // = 0x52
    uint8_t protocol_version;    // = 0x02
    uint8_t data_size;
    float   bullet_velocity;
    float   pitch_angle;
    double  yaw_angle;           // 编码器角度 (rad)，不限位
    float   yaw_omega;
    float   chassis_imu_yaw;
    float   chassis_imu_omega;
    uint8_t mark;
    uint8_t color;
    uint8_t auto_aim_switch;
    uint8_t yaw_temperature;     // yaw轴电机温度
    uint8_t crc8;
} McuReceivePacket_C;

// ── IMU 发送包 ──
typedef struct {
    uint8_t  frame_header1;      // = 0xA7
    uint8_t  frame_header2;      // = 0xB6
    uint8_t  frame_header3;      // = 0xC5
    uint8_t  data_size;          // = 0
    uint32_t crc32;
} ImuSendPacket_C;

// ── IMU 接收包 ──
typedef struct {
    uint8_t  frame_header1;      // = 0xA7
    uint8_t  frame_header2;      // = 0xB6
    uint8_t  frame_header3;      // = 0xC5
    uint8_t  data_size;
    float    gx;
    float    gy;
    float    gz;
    float    ax;
    float    ay;
    float    az;
    double   euler_yaw;
    double   euler_pitch;
    double   euler_roll;
    uint32_t dt_one_tenth_ms;
    uint32_t crc32;
} ImuReceivePacket_C;

#pragma pack(pop)

// ── 聚合数据 ──
typedef struct {
    bool              imu_valid;
    ImuReceivePacket_C imu_packet;
    bool              mcu_valid;
    McuReceivePacket_C mcu_packet;
} RobotLatestData_C;

// ── 融合输出（IMU 高频 + MCU 低频）──
typedef struct {
    bool   valid;                // yaw_angle 绝对基准已到位
    double yaw_pos;              // yaw 关节解卷绕位置 (rad, 多圈)
    double yaw_rate;             // yaw 关节速度 (rad/s)
    double chassis_yaw;          // 底盘 world 系 yaw（解卷绕）
    double chassis_pitch;        // 底盘 world 系 pitch (rad)
    double chassis_roll;         // 底盘 world 系 roll (rad)
    double imu_yaw_unwrapped;    // IMU euler yaw 解卷绕 (rad)
} RobotFusedData_C;

// ── 严格反解数据包（独立输出）──
// 所有角度 wrap 到 (-π, π]；始终有效，缺失数据以 0 参与；
// 保证 R_imu = R_chassis·Rz(yaw_pos)·Rx(pitch_angle) 恒成立
typedef struct {
    double imu_euler_yaw;        // 反解输入：IMU 欧拉角（始终为 imu 传来数据）
    double imu_euler_pitch;
    double imu_euler_roll;
    double yaw_pos;              // 反解输入：yaw 关节位置（wrap 后）
    double pitch_angle;          // 反解输入：pitch 关节角（wrap 后）
    double chassis_yaw;          // 严格反解底盘欧拉角（wrap 后）
    double chassis_pitch;
    double chassis_roll;
} RobotStrictPose_C;

// ── 不透明句柄 ──
typedef struct RobotCommHandle RobotCommHandle;

// ============================================================================
// API 函数
// ============================================================================

// 创建通信句柄（自动启动 IMU 和 MCU 的串口监听）
RobotCommHandle* robot_comm_create(void);

// 销毁通信句柄
void robot_comm_destroy(RobotCommHandle* handle);

// 获取最新 IMU 和 MCU 数据（MCU 接收数据会在此处做预处理）
RobotLatestData_C robot_comm_get_latest_data(RobotCommHandle* handle);

// 获取融合输出（高频 yaw 位置/速度 + 底盘姿态）
RobotFusedData_C robot_comm_get_fused_data(RobotCommHandle* handle);

// 获取严格反解数据包（独立输出，始终有效）
RobotStrictPose_C robot_comm_get_strict_pose(RobotCommHandle* handle);

// 发送 MCU 数据（发送前做预处理）
bool robot_comm_send_to_mcu(RobotCommHandle* handle, const McuSendPacket_C* packet);

// 发送 IMU 数据（无预处理）
bool robot_comm_send_to_imu(RobotCommHandle* handle, const ImuSendPacket_C* packet);

// 停止通信
void robot_comm_stop(RobotCommHandle* handle);

// ── yaw MPC 求解（参考序列 + 求解，不发送；返回将要发送的值）──
typedef void* YawMpcHandle_C;

typedef struct {
    double yaw_target_angle;    // 预测位置 → 发送 yaw_target_angle (rad)
    double yaw_target_velocity; // 预测速度 → 发送 yaw_target_velocity (rad/s)
    double yaw_torque;          // 控制力矩 → 发送 yaw_torque (N·m)
    double delayed_target;      // 当前参考（延迟 dt*N 步的目标），供显示
} YawMpcStepResult_C;

// 创建 yaw MPC 求解器（comm: robot_comm_create 返回的句柄）
YawMpcHandle_C yaw_mpc_create(RobotCommHandle* comm, double dt_control, int N,
                              double J, double tau_c, double b, double tau_d,
                              double max_torque, double max_torque_rate,
                              double Q, double R, double Rd, int max_iter);

// 销毁
void yaw_mpc_destroy(YawMpcHandle_C handle);

// 每步调用：只传目标位置（内部读融合状态、维护延迟参考序列、求解）
// 返回将要发送的 yaw_target_angle / yaw_target_velocity / yaw_torque（不发送）
YawMpcStepResult_C yaw_mpc_step(YawMpcHandle_C handle, double target_yaw);

// ── 实车 MCU 控制封装（设置 + 后台 100Hz 发送线程）──
typedef void* McuMpcHandle_C;

typedef struct {
    double yaw_target_angle;    // 最新预测位置 (rad)
    double yaw_target_velocity; // 最新预测速度 (rad/s)
    double yaw_torque;          // 最新控制力矩 (N·m)
    double delayed_target;      // 当前参考（延迟 dt*N 步的目标）
} McuMpcState_C;

// 创建封装并自动启动后台 100Hz 发送线程
McuMpcHandle_C mcu_mpc_create(RobotCommHandle* comm, double dt_control, int N,
                              double J, double tau_c, double b, double tau_d,
                              double max_torque, double max_torque_rate,
                              double Q, double R, double Rd, int max_iter);

// 销毁（停止并 join 后台线程）
void mcu_mpc_destroy(McuMpcHandle_C handle);

// 设置发送参数 + mpc 目标（target_yaw 给 mpc，其余由封装类维护；
// target_yaw 自动转换到与 imu_yaw_unwrapped 同一圈内的值）
void mcu_mpc_step(McuMpcHandle_C handle,
                  uint8_t auto_aim_enable, uint8_t yaw_torque_only_mode,
                  double target_yaw, float pitch_target_angle, uint8_t fire);

// 获取最新 mpc 结果（供显示）
McuMpcState_C mcu_mpc_get_state(McuMpcHandle_C handle);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_COMMUNICATION_C_H
