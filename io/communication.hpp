#ifndef IO__COMMUNICATION_HPP
#define IO__COMMUNICATION_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "communication/Com.h"      // SerialCommunicationClass / SerialData / MCUDataFrame
#include "communication/HeadIMU.h"  // HeadIMUSerialCommunicationClass / HeadIMUSerialData

namespace io
{
// 下发给电控的云台命令
struct Command
{
  bool fire = false;  // 是否开火
  float pitch = 0.0f; // rad
  float yaw = 0.0f;   // rad
};

// 主循环用的传感器状态（由 Communication::state_at(t) 返回，已做延迟对齐）
struct State
{
  double bullet_velocity = 0.0;
  std::string enemy_color = "BLUE";
  double pitch_rad = 0.0;      // 延迟对齐后的云台俯仰（rad）
  double yaw_rad = 0.0;        // 延迟对齐后的云台偏航 [-pi, pi]
  double total_yaw_rad = 0.0;  // 累计圈数后的偏航
  double roll_rad = 0.0;       // 横滚（HeadIMU）
  bool use_head_imu = true;
  bool mcu_yaw_online = true;
  double to_mcu_delta_yaw = 0.0;
  double to_mcu_delta_pitch = 0.0;
};

// 串口（MCU）+ HeadIMU 的统一入口，对应原 ArmorDetect_Node 里
// serial_communication_ / headIMUInfos / serial_infos_delay_ 那套逻辑。
class Communication
{
public:
  explicit Communication(const std::string & config_path);
  ~Communication();
  Communication(const Communication &) = delete;
  Communication & operator=(const Communication &) = delete;

  // 主循环：取延迟对齐后的传感器状态（原 processImage 里的延迟对齐段）
  State state_at(std::chrono::steady_clock::time_point t);

  // 下发云台命令（原 sendData）
  void send(const Command & command);
  void send(float pitch, float yaw, bool fire);

  // 打开后每帧把实际下发的指令写入 debug 日志（由 GimbalIo 按 log_send_commands 配置透传）
  void setLogSendCommands(bool enable);

  // 开启自瞄时校准 HeadIMU（原 recalibrateHeadIMU）
  void recalibrate_head_imu();

private:
  void serial_data_callback(const SerialData & msg);
  void head_imu_data_callback(const HeadIMUSerialData & msg);

  // 延迟对齐数据（原 ArmorDetect_Node::DelayInfos）
  struct DelayInfos
  {
    float last_pitch_rad_ = 0;
    float last_yaw_rad_ = 0;
    float last_roll_rad_ = 0;
    float total_yaw_rad_ = 0;
    std::chrono::steady_clock::time_point push_time;
  };

  // 配置
  int fix_enemy_color_ = -1;
  float fix_bullet_velocity_ = -1.0f;
  float bullet_velocity_ = 0.0f;
  float serial_delay_time_ = 0.0f;  // ms
  bool use_head_imu_ = true;

  // MCU 姿态状态
  float last_pitch_rad_mcu_ = 0;
  float last_yaw_rad_mcu_ = 0;
  float total_yaw_rad_mcu_ = 0;
  int current_yaw_circle_mcu_ = 0;

  // HeadIMU 姿态状态
  float last_pitch_rad_imu_ = 0;
  float last_yaw_rad_imu_ = 0;
  float total_yaw_rad_imu_ = 0;
  float last_roll_rad_imu_ = 0;
  int current_yaw_circle_imu_ = 0;

  // MCU/IMU 联合状态
  bool mcu_yaw_online_ = true;
  float head_imu_yaw_ = 0;
  float head_imu_pitch_ = 0;
  float head_imu_roll_ = 0;
  float mcu_yaw_ = 0;
  float mcu_pitch_ = 0;
  float last_mcu_yaw_ = 0;
  float latest_head_imu_yaw_when_mcu_yaw_update_ = 0;
  std::chrono::steady_clock::time_point last_mcu_yaw_update_time_;
  float last_mcu_command_yaw_ = 0;
  float latest_mcu_command_yaw_when_mcu_yaw_update_ = 0;
  float to_mcu_delta_yaw_ = 0;
  float to_mcu_delta_pitch_ = 0;

  // 敌方颜色
  std::string enemy_color_ = "BLUE";

  // 延迟队列（回调线程写、主循环读，用 mutex 保护；原代码未加锁）
  std::queue<DelayInfos> serial_infos_delay_;
  std::mutex delay_mutex_;

  // 硬件对象与线程
  std::shared_ptr<SerialCommunicationClass> serial_;
  std::shared_ptr<HeadIMUSerialCommunicationClass> head_imu_;
  std::thread serial_thread_;
  std::thread head_imu_thread_;
};

}  // namespace io

#endif  // IO__COMMUNICATION_HPP
