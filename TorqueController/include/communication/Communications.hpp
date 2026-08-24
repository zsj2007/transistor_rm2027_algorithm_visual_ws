// Communications.hpp — 基于 SerialProtocol 模板的具体通信类型定义
//
// McuCommunication : 与电控（MCU）通信，CRC8，前导 0x42 0x52 0x02
// ImuCommunication  : 与 IMU 模块通信，CRC32，前导 0xA7 0xB6 0xC5
//
#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "SerialProtocol.hpp"
#include "Protocol.hpp"
#include "CRC.h"
#include "McuDataPreprocessor.h"
#include "FusionFilter.h"
#include <string>
#include <mutex>

// ── 端口筛选函数 ──

// 电控（MCU）：选择不是 IMU 的串口
inline bool mcuPortSelector(const std::string& product_info) {
    return product_info != "AutoAim_IMU_Com";
}

// IMU：选择是 IMU 的串口
inline bool imuPortSelector(const std::string& product_info) {
    return product_info == "AutoAim_IMU_Com";
}

// ── 具体通信类型别名 ──

// 电控（MCU）通信：CRC8，前导字节见 mcu::SendPacket / mcu::ReceivePacket
using McuCommunication = SerialProtocol<
    mcu::SendPacket,
    mcu::ReceivePacket,
    CRC8_Check_Sum,
    mcuPortSelector,
    mcu::PREAMBLE_SIZE
>;

// IMU 通信：CRC32，前导字节见 imu::SendPacket / imu::ReceivePacket
using ImuCommunication = SerialProtocol<
    imu::SendPacket,
    imu::ReceivePacket,
    CRC32_Calculate,
    imuPortSelector,
    imu::PREAMBLE_SIZE
>;

// ============================================================================
// RobotCommunication — 组合 MCU 与 IMU 通信，封装数据预处理
// ============================================================================
// - 回调中直接存储预处理后的 MCU 数据（latest_mcu_packet_ 不再存原始数据）
// - McuDataPreprocessor 在接收回调与发送 MCU 数据 (sendToMcu) 时使用
// - 提供统一的最新数据获取接口和分离的发送接口
// ============================================================================
class RobotCommunication {
public:
    struct LatestData {
        bool               imu_valid = false;
        imu::ReceivePacket imu_packet{};
        bool               mcu_valid = false;
        mcu::ReceivePacket mcu_packet{};
    };

    // 融合权重使用 YawChassisFusion 构造默认值（imu_weight=1.0）
    // mcu_linear_params：MCU 数据线性映射标定参数（默认使用当前标定值）
    explicit RobotCommunication(const McuDataPreprocessor::LinearParams& mcu_linear_params = McuDataPreprocessor::LinearParams{})
        : preprocessor_(mcu_linear_params)
        , mcu_serial_([this](const mcu::ReceivePacket& pkt) { onMcuReceive(pkt); }, false)
        , imu_serial_([this](const imu::ReceivePacket& pkt) { onImuReceive(pkt); }, false)
        , fusion_()
    {
        mcu_serial_.startWorker();
        imu_serial_.startWorker();
    }

    ~RobotCommunication() {
        mcu_serial_.stopWorker();
        imu_serial_.stopWorker();
    }

    // 获取最新数据（MCU 数据已在回调中预处理）
    LatestData getLatestData() {
        LatestData data;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (has_imu_data_) {
                data.imu_packet = latest_imu_packet_;
                data.imu_valid  = true;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mcu_mutex_);
            if (has_mcu_data_) {
                data.mcu_packet = latest_mcu_packet_;   // 已预处理
                data.mcu_valid  = true;
            }
        }
        return data;
    }

    // 发送 MCU 数据（发送前预处理）
    bool sendToMcu(mcu::SendPacket packet) {
        mcu::SendPacket processed = preprocessor_.processSend(packet);
        return mcu_serial_.sendData(processed);
    }

    // 发送 IMU 数据（心跳等，无预处理）
    bool sendToImu(imu::SendPacket packet) {
        return imu_serial_.sendData(packet);
    }

    void stop() {
        mcu_serial_.stopWorker();
        imu_serial_.stopWorker();
    }

    // 融合输出：高频 yaw 关节解卷绕位置/速度 + 底盘 world 系姿态
    // （IMU 高频积分 + MCU yaw 绝对位置校正，线程安全）
    YawChassisFusion::Output getFused() const {
        return fusion_.output();
    }

    // 重新锚定 imu_yaw_unwrapped 到与 yaw_pos 夹角最近的圈内（线程安全）
    void reanchorImuYaw() {
        fusion_.reanchorImuYaw();
    }

    // 严格反解数据包（独立输出；始终有效，缺失数据以 0 参与，角度 wrap 到 (-π, π]）
    YawChassisFusion::StrictPose getStrictPose() const {
        return fusion_.strictPose();
    }

private:
    // ── 回调：存储原始数据 + 喂融合滤波器 ──
    void onImuReceive(const imu::ReceivePacket& packet) {
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            latest_imu_packet_ = packet;
            has_imu_data_      = true;
        }
        fusion_.onImu(packet.euler_yaw, packet.euler_pitch, packet.euler_roll,
                      packet.gx, packet.gy, packet.gz);
    }

    void onMcuReceive(const mcu::ReceivePacket& packet) {
        // 预处理后直接存储（latest_mcu_packet_ 不再存原始数据）
        mcu::ReceivePacket processed = preprocessor_.processReceive(packet);
        {
            std::lock_guard<std::mutex> lock(mcu_mutex_);
            latest_mcu_packet_ = processed;
            has_mcu_data_      = true;
        }
        // 喂融合滤波器的数据同样为预处理后的（与 latest_mcu_packet_ 语义一致）
        fusion_.onMcu(processed.yaw_angle, processed.yaw_omega, processed.pitch_angle,
                      processed.chassis_imu_yaw, processed.chassis_imu_omega);
    }

    // ── 成员变量 ──
    McuDataPreprocessor  preprocessor_;   // MCU 数据线性标定预处理
    McuCommunication mcu_serial_;
    ImuCommunication imu_serial_;

    std::mutex         imu_mutex_;
    imu::ReceivePacket latest_imu_packet_{};
    bool               has_imu_data_ = false;

    std::mutex         mcu_mutex_;
    mcu::ReceivePacket latest_mcu_packet_{};
    bool               has_mcu_data_ = false;

    YawChassisFusion   fusion_;   // IMU 高频 + MCU 低频融合滤波器
};

#endif // COMMUNICATIONS_HPP