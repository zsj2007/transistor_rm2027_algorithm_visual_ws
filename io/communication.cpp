#define _USE_MATH_DEFINES  // M_PI
#include "io/communication.hpp"

#include <cmath>
#include <unistd.h>  // usleep

#include "tools/yaml.hpp"

namespace io
{
Communication::Communication(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  fix_enemy_color_ = tools::read<int>(yaml, "FIX_ENEMY_COLOR");
  if (yaml["FIX_BULLET_VELOCITY"]) fix_bullet_velocity_ = yaml["FIX_BULLET_VELOCITY"].as<float>();
  bullet_velocity_ = tools::read<float>(yaml, "bullet_velocity_");
  serial_delay_time_ = tools::read<float>(yaml, "serial_delay_time");
  if (yaml["use_head_imu"]) use_head_imu_ = yaml["use_head_imu"].as<bool>();

  // 原节点接线：串口 + HeadIMU 各自开一个 timer 线程
  std::string serial_port;
  if (yaml["serial_port"]) serial_port = yaml["serial_port"].as<std::string>();
  serial_ = std::make_shared<SerialCommunicationClass>(
    std::bind(&Communication::serial_data_callback, this, std::placeholders::_1),
    serial_port);
  head_imu_ = std::make_shared<HeadIMUSerialCommunicationClass>(
    std::bind(&Communication::head_imu_data_callback, this, std::placeholders::_1));

  serial_thread_ = std::thread([this] { serial_->timerThread(); });
  head_imu_thread_ = std::thread([this] { head_imu_->timerThread(); });

  // 原节点：初始化时向下位机发一帧零命令
  serial_->sendData(0, 0, false);

  // 初始化延迟队列（原 init_serial_infos）
  DelayInfos init;
  init.push_time = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(delay_mutex_);
    serial_infos_delay_.push(init);
  }
  last_mcu_yaw_update_time_ = std::chrono::steady_clock::now();
}

Communication::~Communication()
{
  if (serial_) serial_->stop();
  if (head_imu_) head_imu_->stop();
  if (serial_thread_.joinable()) serial_thread_.join();
  if (head_imu_thread_.joinable()) head_imu_thread_.join();
}

void Communication::send(float pitch, float yaw, bool fire)
{
  serial_->sendData(pitch, yaw, fire);
  std::lock_guard<std::mutex> lock(delay_mutex_);
  last_mcu_command_yaw_ = yaw;
}

void Communication::setLogSendCommands(bool enable)
{
  if (serial_) serial_->setLogSendCommands(enable);
}

void Communication::send(const Command & command)
{
  send(command.pitch, command.yaw, command.fire);
}

State Communication::state_at(std::chrono::steady_clock::time_point t)
{
  // 原 processImage：MCU 云台角超过 3s 未更新且命令角偏差过大 → 判离线
  if (
    std::chrono::duration_cast<std::chrono::milliseconds>(t - last_mcu_yaw_update_time_).count() > 3000) {
    if (std::fabs(last_mcu_command_yaw_ - latest_mcu_command_yaw_when_mcu_yaw_update_) > 5.0 * M_PI / 180.0) {
      mcu_yaw_online_ = false;
    }
  }

  // 延迟对齐：弹出超过 serial_delay_time 的旧数据，取队首
  std::lock_guard<std::mutex> lock(delay_mutex_);
  while (
    serial_infos_delay_.size() > 1 &&
    std::chrono::duration_cast<std::chrono::milliseconds>(t - serial_infos_delay_.front().push_time).count() >
      serial_delay_time_) {
    serial_infos_delay_.pop();
  }
  const DelayInfos & delayed = serial_infos_delay_.front();

  State state;
  state.bullet_velocity = bullet_velocity_;
  state.enemy_color = enemy_color_;
  state.pitch_rad = delayed.last_pitch_rad_;
  state.yaw_rad = delayed.last_yaw_rad_;
  state.total_yaw_rad = delayed.total_yaw_rad_;
  state.roll_rad = delayed.last_roll_rad_;
  state.use_head_imu = use_head_imu_;
  state.mcu_yaw_online = mcu_yaw_online_;
  state.to_mcu_delta_yaw = to_mcu_delta_yaw_;
  state.to_mcu_delta_pitch = to_mcu_delta_pitch_;
  return state;
}

void Communication::recalibrate_head_imu()
{
  float start_yaw, start_pitch;
  {
    std::lock_guard<std::mutex> lock(delay_mutex_);
    start_yaw = last_yaw_rad_imu_ + to_mcu_delta_yaw_;
    start_pitch = serial_infos_delay_.empty() ? 0.0f : serial_infos_delay_.front().last_pitch_rad_;
  }

  for (int i = 0; i < 20; i++) {
    serial_->sendData(0.0, start_yaw, false);
    usleep(30 * 1000);
  }

  float new_yaw;
  {
    std::lock_guard<std::mutex> lock(delay_mutex_);
    new_yaw = last_yaw_rad_imu_;
    to_mcu_delta_yaw_ = -(new_yaw - start_yaw);
  }
  serial_->sendData(start_pitch, start_yaw + to_mcu_delta_yaw_, false);
}

void Communication::serial_data_callback(const SerialData & msg)
{
  SerialData processed = msg;
  if (fix_enemy_color_ == 0 || fix_enemy_color_ == 1) processed.color = fix_enemy_color_;
  if (fix_bullet_velocity_ >= 0.0f) processed.bullet_velocity = fix_bullet_velocity_;

  // 原 serialDataCallback：bullet_angle 的 1.8 对应约 30°
  float current_pitch_ = ((float)(processed.bullet_angle)) * 30.0f / 1.8f * M_PI / 180.0f;
  float current_yaw_ = ((float)(processed.gimbal_yaw)) * M_PI / 4096.0f;

  std::lock_guard<std::mutex> lock(delay_mutex_);
  bullet_velocity_ = processed.bullet_velocity;
  mcu_yaw_ = current_yaw_;
  mcu_pitch_ = current_pitch_;

  if (last_mcu_yaw_ != mcu_yaw_) {
    latest_head_imu_yaw_when_mcu_yaw_update_ = head_imu_yaw_;
    last_mcu_yaw_ = current_yaw_;
    last_mcu_yaw_update_time_ = std::chrono::steady_clock::now();
    mcu_yaw_online_ = true;
    latest_mcu_command_yaw_when_mcu_yaw_update_ = last_mcu_command_yaw_;
    to_mcu_delta_yaw_ = mcu_yaw_ - latest_head_imu_yaw_when_mcu_yaw_update_;
  }
  to_mcu_delta_pitch_ = mcu_pitch_ - head_imu_pitch_;

  while (current_yaw_ < -M_PI) current_yaw_ += 2 * M_PI;
  while (current_yaw_ > M_PI) current_yaw_ -= 2 * M_PI;

  enemy_color_ = (processed.color == 0) ? "RED" : "BLUE";

  // 云台圈数累计
  if (current_yaw_ < -M_PI / 2 && last_yaw_rad_mcu_ > M_PI / 2) current_yaw_circle_mcu_ += 1;
  else if (current_yaw_ > M_PI / 2 && last_yaw_rad_mcu_ < -M_PI / 2) current_yaw_circle_mcu_ -= 1;

  total_yaw_rad_mcu_ = current_yaw_circle_mcu_ * 2 * M_PI + current_yaw_;
  last_pitch_rad_mcu_ = current_pitch_;
  last_yaw_rad_mcu_ = current_yaw_;

  if (!use_head_imu_) {
    DelayInfos now;
    now.last_pitch_rad_ = last_pitch_rad_mcu_;
    now.last_yaw_rad_ = last_yaw_rad_mcu_;
    now.last_roll_rad_ = 0.0f;
    now.total_yaw_rad_ = total_yaw_rad_mcu_;
    now.push_time = std::chrono::steady_clock::now();
    serial_infos_delay_.push(now);
  }
}

void Communication::head_imu_data_callback(const HeadIMUSerialData & msg)
{
  float current_pitch_ = msg.euler_pitch;
  float current_yaw_ = msg.euler_yaw;
  float current_roll_ = msg.euler_roll;

  std::lock_guard<std::mutex> lock(delay_mutex_);
  head_imu_yaw_ = current_yaw_;
  head_imu_pitch_ = current_pitch_;
  head_imu_roll_ = current_roll_;
  to_mcu_delta_pitch_ = mcu_pitch_ - head_imu_pitch_;

  while (current_yaw_ < -M_PI) current_yaw_ += 2 * M_PI;
  while (current_yaw_ > M_PI) current_yaw_ -= 2 * M_PI;

  if (current_yaw_ < -M_PI / 2 && last_yaw_rad_imu_ > M_PI / 2) current_yaw_circle_imu_ += 1;
  else if (current_yaw_ > M_PI / 2 && last_yaw_rad_imu_ < -M_PI / 2) current_yaw_circle_imu_ -= 1;

  total_yaw_rad_imu_ = current_yaw_circle_imu_ * 2 * M_PI + current_yaw_;
  last_pitch_rad_imu_ = current_pitch_;
  last_yaw_rad_imu_ = current_yaw_;
  last_roll_rad_imu_ = current_roll_;

  if (use_head_imu_) {
    DelayInfos now;
    now.last_pitch_rad_ = last_pitch_rad_imu_;
    now.last_roll_rad_ = last_roll_rad_imu_;
    now.last_yaw_rad_ = last_yaw_rad_imu_;
    now.total_yaw_rad_ = total_yaw_rad_imu_;
    now.push_time = std::chrono::steady_clock::now();
    serial_infos_delay_.push(now);
  }
}

}  // namespace io
