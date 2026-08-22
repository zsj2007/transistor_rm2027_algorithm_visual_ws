// users/torque_manual_test.cpp — TorqueController 手动测试程序
//
// 手动输入 yaw/pitch 目标角（度），TorqueController 内部 MPC 结算 yaw 力矩，
// 由后台 100Hz 线程组装 MCU 帧下发到电控。用于上车前验证力矩控制效果。
//
// 用法: ./build/torque_manual_test [configs/infantry.yaml]
// 注意: 会占用 MCU + IMU 两个串口，运行前请先停掉 infantry / visualizer。
//
// 交互命令（每行可多个，空格分隔；也支持 key=value 形式）:
//   y 30        设置 yaw 目标角（度，绝对角，可多圈）
//   p -5        设置 pitch 目标角（度）
//   f 1 / f 0   开火 / 停火
//   a 1 / a 0   自瞄使能（默认 1；torque 协议里与旧 reset 相反）
//   m 1 / m 0   仅力矩模式
//   i 1 / i 0   yaw 力矩积分补偿
//   s           打印当前状态
//   h           帮助
//   q / Ctrl+C  退出

#include <atomic>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "RobotController.h"

#include "tools/yaml.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

std::atomic<bool> g_running{true};

void printState(const RobotController::State & s)
{
  std::printf(
    "[state] fused=%d mcu=%d imu=%d | yaw_pos=%+7.2f° yaw_rate=%+7.2f°/s "
    "pitch(mcu)=%+6.2f° | target=%+7.2f° delayed=%+7.2f° torque=%+.4f | temp=%d\n",
    static_cast<int>(s.fused.valid), static_cast<int>(s.mcu.valid),
    static_cast<int>(s.imu.valid),
    s.fused.yaw_pos / kDeg2Rad, s.fused.yaw_rate / kDeg2Rad,
    s.mcu.pitch_angle / kDeg2Rad,
    s.mpc.yaw_target_angle / kDeg2Rad, s.mpc.delayed_target / kDeg2Rad,
    s.mpc.yaw_torque, s.mcu.yaw_temperature);
}

void printHelp()
{
  std::printf(
    "命令: y <deg> 设置yaw | p <deg> 设置pitch | f 1/0 开火 | a 1/0 自瞄使能\n"
    "      m 1/0 仅力矩 | i 1/0 积分补偿 | s 打印状态 | h 帮助 | q 退出\n");
}

}  // namespace

int main(int argc, char * argv[])
{
  const std::string config_path =
    argc > 1 ? argv[1] : std::string("configs/infantry.yaml");
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

  McuDataPreprocessor::LinearParams linear{};
  if (tc["send_pitch_scale"]) linear.send_pitch_scale = tc["send_pitch_scale"].as<double>();
  if (tc["send_pitch_offset"]) linear.send_pitch_offset = tc["send_pitch_offset"].as<double>();
  if (tc["recv_pitch_scale"]) linear.recv_pitch_scale = tc["recv_pitch_scale"].as<double>();
  if (tc["recv_pitch_offset"]) linear.recv_pitch_offset = tc["recv_pitch_offset"].as<double>();

  std::printf("=== torque_manual_test ===\n");
  std::printf("配置: %s | dt=%.3fs N=%d J=%.5f tau_c=%.5f b=%.5f\n",
    config_path.c_str(), dt_control, mpc_pred_N, J, tau_c, b);
  std::printf("警告: 会占用 MCU+IMU 串口，请先停掉 infantry/visualizer\n");

  RobotController rc(
    dt_control, mpc_pred_N, J, tau_c, b, tau_d,
    max_torque, max_torque_rate, Q, R, Rd, max_iter,
    integral_gain, linear, /*sequence_mode=*/false);

  // 初始目标：0 / 0，自瞄使能开
  bool auto_aim_enable = true;
  bool yaw_torque_only_mode = false;
  bool integral_enable = false;
  bool fire = false;
  double target_yaw_deg = 0.0;
  double target_pitch_deg = 0.0;
  rc.set(auto_aim_enable, yaw_torque_only_mode,
         target_yaw_deg * kDeg2Rad, target_pitch_deg * kDeg2Rad, fire, integral_enable);

  printHelp();
  std::string line;
  while (g_running) {
    std::printf("> ");
    std::fflush(stdout);
    if (!std::getline(std::cin, line)) break;
    std::istringstream iss(line);
    std::string tok;
    try {
      while (iss >> tok) {
        const char key = tok[0];
        // 支持 "p 1"（空格分开）和 "p=1" / "p1"（粘在一起）两种写法
        auto value = [&iss, &tok]() -> double {
          if (tok.size() > 1) {
            const size_t eq = tok.find('=');
            return std::stod(eq == std::string::npos ? tok.substr(1) : tok.substr(eq + 1));
          }
          std::string next;
          if (!(iss >> next)) {
            throw std::runtime_error("缺少数值");
          }
          return std::stod(next);
        };
        switch (key) {
          case 'y': target_yaw_deg = value(); break;
          case 'p': target_pitch_deg = value(); break;
          case 'f': fire = value() != 0.0; break;
          case 'a': auto_aim_enable = value() != 0.0; break;
          case 'm': yaw_torque_only_mode = value() != 0.0; break;
          case 'i': integral_enable = value() != 0.0; break;
          case 's': printState(rc.getState()); break;
          case 'h': printHelp(); break;
          case 'q': g_running = false; break;
          default: std::printf("未知命令: %s (h 查看帮助)\n", tok.c_str());
        }
      }
    } catch (const std::exception & e) {
      std::printf("输入错误: %s（示例: y 30 / p -5 / f 1）\n", e.what());
    }
    rc.set(auto_aim_enable, yaw_torque_only_mode,
           target_yaw_deg * kDeg2Rad, target_pitch_deg * kDeg2Rad, fire, integral_enable);
  }

  g_running = false;
  std::printf("退出。\n");
  return 0;
}
