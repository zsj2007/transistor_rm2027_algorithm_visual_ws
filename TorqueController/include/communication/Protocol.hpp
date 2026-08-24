#pragma once
#include <cstdint>
#include <cstddef>

// ============================================================================
// MCU (电控) 通信协议
// ============================================================================
namespace mcu {

// 帧同步前导字节数（即 frame_header1 + frame_header2 + protocol_version = 3 字节）
constexpr size_t PREAMBLE_SIZE = 3;

#pragma pack(push, 1)
struct SendPacket
{
    uint8_t frame_header1 = 0x42;
    uint8_t frame_header2 = 0x52;
    uint8_t protocol_version = 0x02;
    uint8_t data_size = 23;             // 1+1+4+1+8+4+4
    uint8_t auto_aim_enable;            // 和之前的reset相反
    uint8_t fire;                       // 火控
    float pitch_target_angle;           // -pi/2 ~ pi/2
    uint8_t yaw_torque_only_mode;       // 仅力矩控制模式
    double yaw_target_angle;            // rad 不限位，计算多圈
    float yaw_target_velocity;          // rad/s
    float yaw_torque;                   // -1.0 ~ 1.0
    uint8_t crc8;
};


struct ReceivePacket
{
    uint8_t frame_header1 = 0x42;
    uint8_t frame_header2 = 0x52;
    uint8_t protocol_version = 0x02;
    uint8_t data_size;
    float bullet_velocity;              // m/s
    float pitch_angle;                  // -pi/2 ~ pi/2
    double yaw_angle;                   // rad 不限位，计算多圈，这里不要减掉imu的角度，直接读yaw轴电机编码器的角度
    float yaw_omega;                    // rad/s ，yaw轴电机编码器读到的角速度
    float chassis_imu_yaw;              // 0 ~ 2pi ，底盘imu积分的yaw轴角度
    float chassis_imu_omega;            // 0 ~ 2pi ，底盘imu的yaw轴角速度
    uint8_t mark;                       // 原递增循环标志位
    uint8_t color;                      // 原颜色标志位
    uint8_t auto_aim_switch;            // 电控的自瞄开关
    uint8_t yaw_temperature;            // yaw轴电机温度
    uint8_t crc8;
};
#pragma pack(pop)

} // namespace mcu


// ============================================================================
// IMU 通信协议
// ============================================================================
namespace imu {

// 帧同步前导字节数（即 frame_header1 + frame_header2 + frame_header3 = 3 字节）
constexpr size_t PREAMBLE_SIZE = 3;

#pragma pack(push, 1)
struct SendPacket
{
    uint8_t frame_header1 = 0xA7;
    uint8_t frame_header2 = 0xB6;
    uint8_t frame_header3 = 0xC5;
    uint8_t data_size = 0;              // 无数据载荷，仅心跳
    uint32_t crc32;
};

struct ReceivePacket
{
    uint8_t frame_header1 = 0xA7;
    uint8_t frame_header2 = 0xB6;
    uint8_t frame_header3 = 0xC5;
    uint8_t data_size;
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    double euler_yaw;
    double euler_pitch;
    double euler_roll;
    uint32_t dt_one_tenth_ms;
    uint32_t crc32;
};
#pragma pack(pop)

} // namespace imu