#include "io/gimbal_io.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

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
  float default_bullet_velocity_ = 15.0f;
  std::string default_enemy_color_ = "BLUE";
  double send_pitch_scale_ = 20.523245;   // 与 McuDataPreprocessor::LinearParams 默认一致
  double send_pitch_offset_ = 0.475049;
  double last_pitch_target_deg_ = 0.0;    // 最近一次 set() 的 pitch（视觉→torque 的角度）
  double last_pitch_proto_ = 0.0;         // 线性映射后实际下发的 pitch 协议值

  // ---- 状态反馈延迟对齐（与 serial 通道 serial_delay_time 语义一致）----
  // 后台 200Hz 采样融合状态，state(t) 返回“t - serial_delay_time”时刻的样本，
  // 让弹道解算拿到的姿态与图像同一时刻；仅缓存 fused.valid 后的锚定样本。
  struct TorqueStateSample
  {
    std::chrono::steady_clock::time_point ts;
    io::State state;
  };
  double serial_delay_ms_ = 30.0;
  mutable std::mutex sample_mtx_;
  std::deque<TorqueStateSample> samples_;
  std::atomic<bool> sampler_running_{false};
  std::thread sampler_thread_;

  ~Impl();
  void startSampler();
  io::State snapshotState(const RobotController::State & s) const;
};

TorqueGimbalSender::TorqueGimbalSender(const std::string & config_path)
  : impl_(std::make_unique<Impl>())
{
  auto yaml = tools::load(config_path);
  const YAML::Node tc = yaml["torque_controller"];

  // 与 serial 通道共用同一个延迟对齐配置（ms）
  if (yaml["serial_delay_time"]) {
    impl_->serial_delay_ms_ = yaml["serial_delay_time"].as<double>();
  }

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
  impl_->send_pitch_scale_ = linear_params.send_pitch_scale;
  impl_->send_pitch_offset_ = linear_params.send_pitch_offset;

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

  impl_->startSampler();
}

std::string TorqueGimbalSender::channelName() const
{
  return "torque";
}

void TorqueGimbalSender::send(const GimbalCommand & cmd)
{
  // 直通 McuMpcController::set：后台 100Hz 线程持续取最新目标求解 MPC 并发送。
  // target_yaw 内部自动转换到与 imu_yaw_unwrapped 夹角最小的等效角（多圈语义）。
  impl_->last_pitch_target_deg_ = cmd.pitch * 180.0 / M_PI;
  impl_->last_pitch_proto_ =
    impl_->send_pitch_scale_ * cmd.pitch + impl_->send_pitch_offset_;
  impl_->rc->set(
    cmd.auto_aim_enable,
    cmd.yaw_torque_only_mode,
    cmd.yaw,
    cmd.pitch,
    cmd.fire,
    cmd.integral_enable);
}

TorqueGimbalSender::Impl::~Impl()
{
  sampler_running_ = false;
  if (sampler_thread_.joinable()) sampler_thread_.join();
}

// 后台采样：200Hz 缓存“当前”融合状态，供 state(t) 做延迟对齐
void TorqueGimbalSender::Impl::startSampler()
{
  sampler_running_ = true;
  sampler_thread_ = std::thread([this] {
    while (sampler_running_) {
      const auto start = std::chrono::steady_clock::now();
      const io::State st = snapshotState(rc->getState());
      if (st.mcu_yaw_online) {  // 只缓存融合已锚定的有效样本
        std::lock_guard<std::mutex> lock(sample_mtx_);
        samples_.push_back(TorqueStateSample{start, st});
        while (samples_.size() > 1 &&
               start - samples_.front().ts > std::chrono::seconds(1)) {
          samples_.pop_front();
        }
      }
      std::this_thread::sleep_until(start + std::chrono::milliseconds(5));
    }
  });
}

// 从 RobotController 状态映射为 io::State；mcu_yaw_online 做 fused.valid 门控
io::State TorqueGimbalSender::Impl::snapshotState(const RobotController::State & s) const
{
  io::State st;

  // 全走 strict 严格反解包：始终有效，缺失数据以 0 参与，不做 valid 分支
  st.pitch_rad = s.strict.imu_euler_pitch;
  st.total_yaw_rad = s.fused.imu_yaw_unwrapped;
  st.yaw_rad = std::remainder(st.total_yaw_rad, kTwoPi);  // wrap 到 (-pi, pi]
  st.roll_rad = s.strict.imu_euler_roll;

  // strict 包里不含弹速/颜色：有 MCU 数据用实时值，否则回默认兜底
  if (s.mcu.valid) {
    st.bullet_velocity = s.mcu.bullet_velocity;
    st.enemy_color = (s.mcu.color == 1) ? "BLUE" : "RED";
  } else {
    st.bullet_velocity = default_bullet_velocity_;
    st.enemy_color = default_enemy_color_;
  }

  st.use_head_imu = false;          // 融合由 TorqueController 内部完成
  st.mcu_yaw_online = s.fused.valid;  // 有效性门控：融合未锚定（无 MCU yaw）视为离线
  st.to_mcu_delta_yaw = 0.0;
  st.to_mcu_delta_pitch = 0.0;
  return st;
}

io::State TorqueGimbalSender::state(std::chrono::steady_clock::time_point t) const
{
  const auto s = impl_->rc->getState();
  const io::State fresh = impl_->snapshotState(s);

  std::lock_guard<std::mutex> lock(impl_->sample_mtx_);
  if (impl_->samples_.empty()) {
    // 融合未锚定：返回当前映射（mcu_yaw_online=false，下游据此判定未就绪）
    return fresh;
  }

  // 延迟对齐：返回 t - serial_delay_time 时刻最接近的样本（与 serial 通道一致）
  const auto target =
    t - std::chrono::milliseconds(static_cast<int64_t>(impl_->serial_delay_ms_));
  const TorqueGimbalSender::Impl::TorqueStateSample * best = nullptr;
  auto best_dist = std::chrono::steady_clock::duration::max();
  for (const auto & smp : impl_->samples_) {
    const auto dist = (smp.ts > target) ? (smp.ts - target) : (target - smp.ts);
    if (dist < best_dist) {
      best_dist = dist;
      best = &smp;
    }
  }
  return best ? best->state : fresh;
}

TorqueDebugState TorqueGimbalSender::torqueDebugState() const
{
  TorqueDebugState d;
  d.valid = true;
  const auto s = impl_->rc->getState();
  d.fused_valid = s.fused.valid;
  d.mcu_valid = s.mcu.valid;
  d.imu_valid = s.imu.valid;
  d.yaw_pos_deg = s.fused.yaw_pos * 180.0 / M_PI;
  d.yaw_rate_deg_s = s.fused.yaw_rate * 180.0 / M_PI;
  d.imu_yaw_deg = s.fused.imu_yaw_unwrapped * 180.0 / M_PI;
  d.yaw_torque = s.mpc.yaw_torque;
  d.yaw_target_angle_deg = s.mpc.yaw_target_angle * 180.0 / M_PI;
  d.yaw_target_velocity = s.mpc.yaw_target_velocity;
  d.delayed_target_deg = s.mpc.delayed_target * 180.0 / M_PI;
  d.integral = s.mpc.integral;
  d.pitch_target_deg = impl_->last_pitch_target_deg_;
  d.pitch_proto = impl_->last_pitch_proto_;
  if (s.mcu.valid) {
    d.mcu_pitch_deg = s.mcu.pitch_angle * 180.0 / M_PI;
    d.mcu_yaw_deg = s.mcu.yaw_angle * 180.0 / M_PI;
  }
  return d;
}
#endif  // IO_ENABLE_TORQUE_CONTROLLER

// ==================== 可切换下发模块 ====================

GimbalIo::GimbalIo(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  channel_name_ = yaml["command_channel"] ? yaml["command_channel"].as<std::string>() : "serial";
  if (yaml["log_send_commands"]) {
    log_send_commands_ = yaml["log_send_commands"].as<bool>();
  }

  if (channel_name_ == "serial") {
    comm_ = std::make_unique<io::Communication>(config_path);
    comm_->setLogSendCommands(log_send_commands_);
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
    return torque_->state(t);
  }
#endif
  return comm_->state_at(t);
}

void GimbalIo::send(const GimbalCommand & cmd)
{
  sender_->send(cmd);

  if (!log_send_commands_) return;

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kJumpAlertDeg = 10.0;  // 相邻两帧命令角跳变超过该值即告警（本地抓“剧烈扭转”用）

  const double yaw_deg = cmd.yaw * 180.0 / kPi;
  const double pitch_deg = cmd.pitch * 180.0 / kPi;

  if (cmd_initialized_) {
    const double dyaw_deg =
      std::remainder(cmd.yaw - last_sent_cmd_.yaw, 2.0 * kPi) * 180.0 / kPi;
    const double dpitch_deg = (cmd.pitch - last_sent_cmd_.pitch) * 180.0 / kPi;
    if (std::fabs(dyaw_deg) > kJumpAlertDeg || std::fabs(dpitch_deg) > kJumpAlertDeg) {
      tools::logger()->warn(
        "[SEND-JUMP] yaw {:+.2f}° -> {:+.2f}° (Δ{:+.2f}°) | pitch {:+.2f}° -> {:+.2f}° (Δ{:+.2f}°)",
        last_sent_cmd_.yaw * 180.0 / kPi, yaw_deg, dyaw_deg,
        last_sent_cmd_.pitch * 180.0 / kPi, pitch_deg, dpitch_deg);
    }
  }
  last_sent_cmd_ = cmd;
  cmd_initialized_ = true;

  // info 级：每帧把实际下发的指令打到控制台（log_send_commands 打开时），同时也会写入日志文件
  tools::logger()->info(
    "[SEND] {} | pitch {:+.2f}° yaw {:+.2f}° fire={} auto_aim_enable={}",
    channel_name_, pitch_deg, yaw_deg,
    static_cast<int>(cmd.fire), static_cast<int>(cmd.auto_aim_enable));
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

TorqueDebugState GimbalIo::torqueDebugState() const
{
#if IO_ENABLE_TORQUE_CONTROLLER
  if (channel_name_ == "torque" && torque_) {
    return torque_->torqueDebugState();
  }
#endif
  return TorqueDebugState{};
}

}  // namespace io
