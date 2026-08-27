#ifndef IO__GIMBAL_IO_HPP
#define IO__GIMBAL_IO_HPP

#include <chrono>
#include <memory>
#include <string>

#include "io/communication.hpp"

namespace io
{
// 统一下发命令：
//   serial 通道只使用 pitch / yaw / fire；
//   torque 通道还使用 auto_aim_enable / yaw_torque_only_mode / integral_enable。
struct GimbalCommand
{
  bool auto_aim_enable = true;    // torque：直通 MCU 的 auto_aim_enable（与旧 reset 相反）
  bool yaw_torque_only_mode = false; // torque：仅力矩控制模式
  float pitch = 0.0f;             // rad
  double yaw = 0.0;               // rad，绝对角（多圈语义；torque 通道内部做最近等效角映射）
  bool fire = false;
  bool integral_enable = false;   // 仅 torque：yaw 力矩积分补偿开关
};

// torque 通道实时调试信息（serial 通道返回 valid=false）。
// 供 infantry_debug 之类的调试程序在画面上实时显示：
//   视觉命令角（发给 torque 之前） + TorqueController 的 MPC 输出（发给电控之后）。
struct TorqueDebugState
{
  bool valid = false;            // 仅 command_channel=torque 时为 true
  bool fused_valid = false;      // 融合有效（已拿到 MCU yaw 锚定）
  bool mcu_valid = false;        // MCU 收到过数据（粘性）
  bool imu_valid = false;        // IMU 收到过数据（粘性）
  double yaw_pos_deg = 0.0;      // 融合 yaw 关节角（MPC 的 theta）
  double yaw_rate_deg_s = 0.0;   // 融合 yaw 角速度
  double imu_yaw_deg = 0.0;      // IMU yaw（解卷绕，度）
  double yaw_torque = 0.0;       // MPC 计算力矩（N·m，发给电控）
  double yaw_target_angle_deg = 0.0; // MPC 预测角（发给电控的 yaw 目标）
  double yaw_target_velocity = 0.0;  // MPC 预测速度（rad/s，发给电控）
  double delayed_target_deg = 0.0;   // MPC 延迟参考（显示用）
  double integral = 0.0;         // yaw 力矩积分补偿值
  double pitch_target_deg = 0.0; // 视觉算出的 pitch 目标（发给 torque 的角度）
  double pitch_proto = 0.0;      // 线性映射后实际下发的 pitch 协议值
  double mcu_pitch_deg = 0.0;    // 电控回传 pitch（已标定，imu_euler_pitch 语义）
  double mcu_yaw_deg = 0.0;      // 电控回传 yaw（编码器多圈角，度）
};

// 下发通道抽象：serial（原串口角度协议）| torque（TorqueController MPC 力矩协议）
class GimbalCommandSender
{
public:
  virtual ~GimbalCommandSender() = default;
  virtual std::string channelName() const = 0;
  virtual void send(const GimbalCommand & cmd) = 0;
};

// serial 通道：包装 io::Communication::send（原下发逻辑，角度协议 0x42 0x52 0xCD）
class SerialGimbalSender final : public GimbalCommandSender
{
public:
  explicit SerialGimbalSender(io::Communication & comm);
  std::string channelName() const override;
  void send(const GimbalCommand & cmd) override;

private:
  io::Communication & comm_;
};

#if IO_ENABLE_TORQUE_CONTROLLER
// torque 通道：包装 TorqueController 的 RobotController。
// 构造时即建立 MCU/IMU 串口 + 融合滤波器 + yaw MPC，并启动后台 100Hz 发送线程；
// 传感器状态（云台角/弹速/颜色）也由它提供（见 state()）。
class TorqueGimbalSender final : public GimbalCommandSender
{
public:
  explicit TorqueGimbalSender(const std::string & config_path);
  std::string channelName() const override;
  void send(const GimbalCommand & cmd) override;

  // 从 RobotController 融合状态映射为 io::State（torque 通道的传感器输入）。
  // 与 serial 通道一致：按帧时间戳 t 做 serial_delay_time 延迟对齐，
  // 且仅在融合有效（fused.valid）后返回锚定状态（mcu_yaw_online 门控）。
  io::State state(std::chrono::steady_clock::time_point t) const;

  // torque 通道实时调试信息（MPC 输出 + 融合/电控状态）
  TorqueDebugState torqueDebugState() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
#endif  // IO_ENABLE_TORQUE_CONTROLLER

// 可切换下发模块：按 config 的 command_channel 选择 serial / torque。
//   serial：内部创建 io::Communication（MCU 串口 + HeadIMU），状态与下发均走旧通道；
//   torque：内部创建 TorqueGimbalSender（RobotController 自管串口 + IMU + 融合 + MPC），
//           不再占用旧串口，状态由 RobotController::getState() 映射而来。
class GimbalIo
{
public:
  explicit GimbalIo(const std::string & config_path);
  ~GimbalIo();
  GimbalIo(const GimbalIo &) = delete;
  GimbalIo & operator=(const GimbalIo &) = delete;

  std::string channelName() const;

  // 取延迟对齐后的传感器状态（serial：io::Communication::state_at；torque：RobotController）
  io::State stateAt(std::chrono::steady_clock::time_point t);

  // 下发云台命令（两个通道统一入口）
  void send(const GimbalCommand & cmd);

  // 开启自瞄时校准 HeadIMU：仅 serial 通道有效；torque 通道为空操作
  void recalibrateHeadImu();

  // 实时调试信息：torque 通道返回 MPC/融合状态，serial 通道返回 valid=false
  TorqueDebugState torqueDebugState() const;

private:
  std::unique_ptr<io::Communication> comm_;        // serial 通道持有
  std::unique_ptr<GimbalCommandSender> sender_;    // 当前通道
#if IO_ENABLE_TORQUE_CONTROLLER
  TorqueGimbalSender * torque_ = nullptr;          // 仅 torque 通道非空（由 sender_ 持有）
#endif
  std::string channel_name_;
  bool log_send_commands_ = false;                 // 每帧把实际下发命令打到控制台（本地视频调试/实车排障用）
  bool cmd_initialized_ = false;                   // 首帧不下发前无“上一帧”可比
  GimbalCommand last_sent_cmd_;                    // 上一帧命令，用于突变告警
};

}  // namespace io

#endif  // IO__GIMBAL_IO_HPP
