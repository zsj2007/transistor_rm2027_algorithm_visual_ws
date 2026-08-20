#include "io/gimbal_io.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

#if IO_ENABLE_TORQUE_CONTROLLER
#include "RobotController.h"
#endif

namespace io
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
}  // namespace

// ==================== serial 通道 ====================

SerialGimbalSender::SerialGimbalSender(io::Communication & comm)
  : comm_(comm)
{
}

std::string SerialGimbalSender::channelName() const
{
  return "serial";
}

void SerialGimbalSender::send(const GimbalCommand & cmd)
{
  comm_.send(cmd.pitch, static_cast<float>(cmd.yaw), cmd.fire);
}

#if IO_ENABLE_TORQUE_CONTROLLER
// ==================== torque 通道 ====================

struct TorqueGimbalSender::Impl
{
  std::unique_ptr<RobotController> rc;
  bool yaw_torque_only_mode_ = false;
  bool integral_enable_ = false;
  float default_bullet_velocity_ = 15.0f;
  std::string default_enemy_color_ = "BLUE";
};

TorqueGimbalSender::TorqueGimbalSender(const std::string & config_path)
  : impl_(std::make_unique<Impl>())
{
  auto yaml = tools::load(config_path);
  const YAML::Node tc = yaml["torque_controller"];

  const double dt_control = tc["dt_control"] ? tc["dt_control"].as<double>() : 0.01;
  const int mpc_pred_N = tc["mpc_pred_N"] ? tc["mpc_pred_N"].as<int>() : 20;
  const double J = tc["J"] ? tc["J"].as<double>() : 0.016541;
  const double tau_c = tc["tau_c"] ? tc["tau_c"].as<double>() : 0.097297;
  const double b = tc["b"] ? tc["b"].as<double>() : 0.0321;
  const double tau_d = tc["tau_d"] ? tc["tau_d"].as<double>() : 0.0;
  const double max_torque = tc["max_torque"] ? tc["max_torque"].as<double>() : 1.0;
  const double max_torque_rate = tc["max_torque_rate"] ? tc["max_torque_rate"].as<double>() : 40.0;
  const double Q = tc["Q"] ? tc["Q"].as<double>() : 5.0;
  const double R = tc["R"] ? tc["R"].as<double>() : 0.01;
  const double Rd = tc["Rd"] ? tc["Rd"].as<double>() : 0.1;
  const int max_iter = tc["max_iter"] ? tc["max_iter"].as<int>() : 30;
  const double integral_gain = tc["integral_gain"] ? tc["integral_gain"].as<double>() : 0.01;
  const bool sequence_mode = tc["sequence_mode"] ? tc["sequence_mode"].as<bool>() : false;

  impl_->yaw_torque_only_mode_ =
    tc["yaw_torque_only_mode"] ? tc["yaw_torque_only_mode"].as<bool>() : false;
  impl_->integral_enable_ =
    tc["integral_enable"] ? tc["integral_enable"].as<bool>() : false;
  if (yaml["bullet_velocity_"]) {
    impl_->default_bullet_velocity_ = yaml["bullet_velocity_"].as<float>();
  }
  if (yaml["init_enemy_color"]) {
    impl_->default_enemy_color_ = yaml["init_enemy_color"].as<std::string>();
  }

  // MCU 数据线性标定（默认 = Infantry1 当前标定，与 McuDataPreprocessor::LinearParams{} 一致）
  McuDataPreprocessor::LinearParams linear_params{};
  if (tc["send_pitch_scale"]) linear_params.send_pitch_scale = tc["send_pitch_scale"].as<double>();
  if (tc["send_pitch_offset"]) linear_params.send_pitch_offset = tc["send_pitch_offset"].as<double>();
  if (tc["recv_pitch_scale"]) linear_params.recv_pitch_scale = tc["recv_pitch_scale"].as<double>();
  if (tc["recv_pitch_offset"]) linear_params.recv_pitch_offset = tc["recv_pitch_offset"].as<double>();

  impl_->rc = std::make_unique<RobotController>(
    dt_control, mpc_pred_N,
    J, tau_c, b, tau_d,
    max_torque, max_torque_rate,
    Q, R, Rd, max_iter,
    integral_gain,
    linear_params,
    sequence_mode);

  tools::logger()->info(
    "TorqueGimbalSender ready (dt={}s N={} J={} tau_c={} b={} tau_d={} "
    "max_torque={} max_torque_rate={} Q={} R={} Rd={} max_iter={})",
    dt_control, mpc_pred_N, J, tau_c, b, tau_d,
    max_torque, max_torque_rate, Q, R, Rd, max_iter);
}

std::string TorqueGimbalSender::channelName() const
{
  return "torque";
}

void TorqueGimbalSender::send(const GimbalCommand & cmd)
{
  // 直通 McuMpcController::set：后台 100Hz 线程持续取最新目标求解 MPC 并发送。
  // target_yaw 内部自动转换到与 imu_yaw_unwrapped 夹角最小的等效角（多圈语义）。
  impl_->rc->set(
    cmd.auto_aim_enable,
    cmd.yaw_torque_only_mode,
    cmd.yaw,
    cmd.pitch,
    cmd.fire,
    cmd.integral_enable);
}

io::State TorqueGimbalSender::state() const
{
  io::State st;
  auto s = impl_->rc->getState();

  if (s.mcu.valid) {
    st.bullet_velocity = s.mcu.bullet_velocity;
    st.pitch_rad = s.mcu.pitch_angle;
    st.total_yaw_rad = s.fused.valid ? s.fused.yaw_pos : s.mcu.yaw_angle;
    st.yaw_rad = std::remainder(st.total_yaw_rad, kTwoPi);  // wrap 到 (-pi, pi]
    st.enemy_color = (s.mcu.color == 1) ? "BLUE" : "RED";
  } else {
    st.bullet_velocity = impl_->default_bullet_velocity_;
    st.enemy_color = impl_->default_enemy_color_;
  }

  st.roll_rad = s.imu.valid ? s.imu.euler_roll : 0.0;
  st.use_head_imu = false;          // 融合由 TorqueController 内部完成
  st.mcu_yaw_online = s.mcu.valid;  // 首包到达前视为离线
  st.to_mcu_delta_yaw = 0.0;
  st.to_mcu_delta_pitch = 0.0;
  return st;
}
#endif  // IO_ENABLE_TORQUE_CONTROLLER

// ==================== 可切换下发模块 ====================

GimbalIo::GimbalIo(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  channel_name_ = yaml["command_channel"] ? yaml["command_channel"].as<std::string>() : "serial";

  if (channel_name_ == "serial") {
    comm_ = std::make_unique<io::Communication>(config_path);
    sender_ = std::make_unique<SerialGimbalSender>(*comm_);
  } else if (channel_name_ == "torque") {
#if IO_ENABLE_TORQUE_CONTROLLER
    auto torque = std::make_unique<TorqueGimbalSender>(config_path);
    torque_ = torque.get();
    sender_ = std::move(torque);
#else
    throw std::runtime_error(
      "command_channel=torque 需要 TorqueController 编译支持 "
      "(IO_ENABLE_TORQUE_CONTROLLER 未开启，检查 Ceres 与 ~/TorqueController)");
#endif
  } else {
    throw std::runtime_error("未知 command_channel: " + channel_name_ + "（可选 serial / torque）");
  }

  tools::logger()->info("GimbalIo channel: {}", channel_name_);
}

GimbalIo::~GimbalIo() = default;

std::string GimbalIo::channelName() const
{
  return channel_name_;
}

io::State GimbalIo::stateAt(std::chrono::steady_clock::time_point t)
{
#if IO_ENABLE_TORQUE_CONTROLLER
  if (channel_name_ == "torque") {
    return torque_->state();
  }
#endif
  return comm_->state_at(t);
}

void GimbalIo::send(const GimbalCommand & cmd)
{
  sender_->send(cmd);
}

void GimbalIo::recalibrateHeadImu()
{
  if (comm_) {
    comm_->recalibrate_head_imu();
  } else {
    tools::logger()->info(
      "GimbalIo: torque 通道由 TorqueController 内部完成 IMU 融合，跳过 HeadIMU 校准");
  }
}

}  // namespace io
