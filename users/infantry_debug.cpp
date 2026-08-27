// users/infantry_debug.cpp — 步兵调试版主程序
//
// 与 infantry.cpp 相同的管线，额外输出：
//   1. 滚动帧率（FrameRateCounter，主循环吞吐，窗口默认 60 帧）
//   2. 流水线结果速率（真正处理完成的帧数/秒）
//   3. 每 report_interval 帧自动打印各阶段耗时（PerformanceMonitor，stdout 报告）
//   4. 实时调试窗口 torque_debug：叠加显示
//        - 自瞄检测/解算出的命令 yaw/pitch（发给 torque 之前）
//        - TorqueController 的 MPC 输出：yaw 力矩 / yaw 目标角 / pitch 目标与协议值
//        - 融合状态（fused/mcu/imu valid、yaw_pos、IMU yaw、电控回传角度）
//
// 用法：./build/infantry_debug configs/infantry.yaml [-headless]
//   - 摄像机模式：configs/infantry.yaml（command_channel=torque 时显示 MPC 输出）
//   - 视频模式：configs/infantry_video.yaml
//   - -headless：不创建 OpenCV 窗口（无显示环境时使用）

#include <chrono>
#include <filesystem>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal_io.hpp"
#include "io/watchdog.hpp"
#include "pipeline/AutoAimPipeline.h"
#include "utils/FrameRateCounter.h"
#include "utils/PerformanceMonitor.h"
#include "utils/ThreadPool.h"
#include "tools/exiter.hpp"
#include "tools/cpu_affinity.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace fs = std::filesystem;

namespace
{
constexpr double kDeg2Rad = 0.017453292519943295;

// 在画面左上角叠加实时调试面板：
//   第 1 行：视觉命令角（发给 torque 之前）
//   第 2~3 行：torque 通道 MPC/下发结果（发给电控之后）
//   第 4~5 行：融合/电控状态（判断云台是否真的响应）
void drawTorqueDebugPanel(
  cv::Mat & img,
  const io::GimbalCommand & cmd,
  const io::TorqueDebugState & t,
  bool has_target,
  float delta_pitch_deg,
  float delta_yaw_deg)
{
  cv::Mat overlay = img.clone();
  cv::rectangle(overlay, cv::Point(10, 10), cv::Point(900, 150),
                cv::Scalar(0, 0, 0), cv::FILLED);
  cv::addWeighted(overlay, 0.55, img, 0.45, 0.0, img);

  const cv::Point org(20, 34);
  const int dy = 22;
  const double fs = 0.52;
  const cv::Scalar kVision(255, 255, 0);   // 青黄：视觉命令
  const cv::Scalar kTorque(0, 255, 255);   // 黄：torque 输出
  const cv::Scalar kFusion(0, 255, 0);     // 绿：融合/反馈
  const cv::Scalar kGray(180, 180, 180);

  const double cmd_yaw_deg = cmd.yaw / kDeg2Rad;
  const double cmd_pitch_deg = cmd.pitch / kDeg2Rad;

  // 视觉：自瞄检测/解算出的命令角（发给 torque 之前）
  cv::putText(
    img,
    cv::format("VISION  yaw %+7.2f deg  pitch %+6.2f deg  (dyaw %+5.2f dpitch %+5.2f)  fire=%d aa=%d%s",
               cmd_yaw_deg, cmd_pitch_deg,
               delta_yaw_deg, delta_pitch_deg,
               static_cast<int>(cmd.fire), static_cast<int>(cmd.auto_aim_enable),
               has_target ? "" : "  NO_TARGET"),
    org, cv::FONT_HERSHEY_SIMPLEX, fs, kVision, 1, cv::LINE_AA);

  // torque：发给电控之后的 MPC 结果
  cv::putText(
    img,
    cv::format("TORQUE  yaw_torque %+6.3f N*m  yaw_target %+7.2f deg  delayed %+7.2f  yaw_vel %+5.2f rad/s",
               t.yaw_torque, t.yaw_target_angle_deg, t.delayed_target_deg,
               t.yaw_target_velocity),
    cv::Point(org.x, org.y + dy), cv::FONT_HERSHEY_SIMPLEX, fs, kTorque, 1, cv::LINE_AA);
  cv::putText(
    img,
    cv::format("         pitch_target %+6.2f deg  proto %7.3f  mcu_pitch %+6.2f deg",
               t.pitch_target_deg, t.pitch_proto, t.mcu_pitch_deg),
    cv::Point(org.x, org.y + 2 * dy), cv::FONT_HERSHEY_SIMPLEX, fs, kTorque, 1, cv::LINE_AA);

  // 融合/反馈：判断云台是否真的响应了命令
  cv::putText(
    img,
    cv::format("FUSION  fused=%d mcu=%d imu=%d  yaw_pos %+7.2f  imu_yaw %+7.2f  mcu_yaw %+7.2f",
               static_cast<int>(t.fused_valid), static_cast<int>(t.mcu_valid),
               static_cast<int>(t.imu_valid),
               t.yaw_pos_deg, t.imu_yaw_deg, t.mcu_yaw_deg),
    cv::Point(org.x, org.y + 3 * dy), cv::FONT_HERSHEY_SIMPLEX, fs, kFusion, 1, cv::LINE_AA);
  cv::putText(
    img,
    cv::format("         yaw_rate %+5.2f deg/s  integral %+.4f  torque_mode=%d",
               t.yaw_rate_deg_s, t.integral, static_cast<int>(cmd.yaw_torque_only_mode)),
    cv::Point(org.x, org.y + 4 * dy), cv::FONT_HERSHEY_SIMPLEX, fs, kFusion, 1, cv::LINE_AA);

  if (!t.valid) {
    cv::putText(
      img, "torque channel disabled (serial)", cv::Point(org.x, org.y + 5 * dy),
      cv::FONT_HERSHEY_SIMPLEX, fs, kGray, 1, cv::LINE_AA);
  }
}

}  // namespace

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{headless | | 不创建 OpenCV 窗口（无显示环境时使用）}"
  "{@config-path   | configs/infantry_video.yaml | 位置参数，yaml配置文件路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  // ---- 配置读取（提前到最前：cpu_pinning 需在创建任何线程前绑好主线程）----
  auto yaml = tools::load(config_path);
  auto config_file_ptr = std::make_shared<YAML::Node>(yaml);
  // CPU 绑核：主线程绑到 E 核（cpu_pinning.other_cores），后续新建线程继承该亲和；
  // OpenVINO 推理线程由 cpu_pinning.yolo_core_type 绑到 P 核（在 OpenvinoInfer 内处理）。
  bool cpu_pinned = tools::cpu_affinity::initFromYaml(yaml);
  tools::logger()->info(
    "[CPU] cpu_pinning {} | main thread allowed cpus: {}",
    cpu_pinned ? "ENABLED" : "DISABLED",
    tools::cpu_affinity::currentCpusAllowedList());

  // 无目标（reset）时的云台行为：false=回 0 位（原行为）；true=保持最后位置不动
  bool no_target_hold_position = yaml["no_target_hold_position"]
                                   ? yaml["no_target_hold_position"].as<bool>()
                                   : false;

  // torque 通道专用：仅力矩模式 / yaw 力矩积分补偿（serial 通道忽略），默认 false
  bool torque_yaw_torque_only_mode = false;
  bool torque_integral_enable = false;
  if (yaml["torque_controller"]) {
    const YAML::Node & tc = yaml["torque_controller"];
    if (tc["yaw_torque_only_mode"]) torque_yaw_torque_only_mode = tc["yaw_torque_only_mode"].as<bool>();
    if (tc["integral_enable"]) torque_integral_enable = tc["integral_enable"].as<bool>();
  }

  tools::Exiter exiter;

  // ---- io：硬件抽象层（构造即初始化）----
  io::Camera camera(config_path);
  io::GimbalIo gimbal(config_path);   // 可切换下发模块：serial / torque
  io::Watchdog watchdog(config_path);

  // ---- 性能监控 + 全局线程池 ----
  auto node_start_time = std::chrono::steady_clock::now();
  fs::path workspace_path = fs::path(config_path).parent_path().parent_path();

  bool perf_enabled = yaml["performance_monitor_enabled"]
                        ? yaml["performance_monitor_enabled"].as<bool>()
                        : true;
  size_t perf_interval = yaml["performance_monitor_report_interval"]
                           ? yaml["performance_monitor_report_interval"].as<size_t>()
                           : 90;
  auto performance_monitor = std::make_shared<PerformanceMonitor>(perf_enabled, perf_interval);

  int thread_pool_size = yaml["thread_pool_size"] ? yaml["thread_pool_size"].as<int>() : 0;
  if (thread_pool_size <= 0) {
    // 0=自动：绑核时按 E 核数量创建线程，避免 18 个 worker 挤在 8 个 E 核上超订
    thread_pool_size = static_cast<int>(tools::cpu_affinity::otherCpuCount());
    if (thread_pool_size <= 0) {
      thread_pool_size = static_cast<int>(std::thread::hardware_concurrency());
    }
  }
  ::utils::threadPool(thread_pool_size);

  AutoAimPipeline pipeline(config_file_ptr, workspace_path, node_start_time, performance_monitor);

  double fx = yaml["camera_matrix"][0][0].as<double>();
  double fy = yaml["camera_matrix"][1][1].as<double>();

  // ---- 调试统计：输入帧率 + 流水线结果速率 ----
  FrameRateCounter loop_fps(60);      // 主循环输入吞吐（受 frame_rate 限制）
  FrameRateCounter result_fps(60);    // 流水线真正完成（取回有效结果）的速率
  const size_t fps_log_interval = 30; // 每 30 帧打一条 FPS
  size_t frame_count = 0;

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  double source_timestamp_s = 0.0;
  bool last_auto_aim_switch = true;
  float last_send_pitch = 0.0f;
  double last_send_yaw = 0.0;
  double frame_rate = yaml["frame_rate"] ? yaml["frame_rate"].as<double>() : 80.0;

  // ---- 实时调试面板状态（-headless 时不创建窗口）----
  const bool headless = cli.has("headless");
  cv::Mat last_debug_frame;                    // 最近一帧画面（带检测绘制）
  io::GimbalCommand last_cmd;                  // 最近一次下发的命令
  bool last_has_target = false;                // 最近一帧是否有目标（reset 之外）
  float last_delta_pitch_deg = 0.0f;           // 弹道解算的 pitch 增量（度）
  float last_delta_yaw_deg = 0.0f;             // 弹道解算的 yaw 增量（度）

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();

    // ① 取帧（阻塞到新帧）
    camera.read(img, t, source_timestamp_s);
    if (img.empty()) continue;

    // ② 传感器状态（延迟对齐后）
    auto state = gimbal.stateAt(t);

    // ③ 自瞄开关 + HeadIMU 校准
    bool auto_aim_switch = true;
    if ((!last_auto_aim_switch && auto_aim_switch) &&
        state.use_head_imu && !state.mcu_yaw_online) {
      gimbal.recalibrateHeadImu();
    }
    last_auto_aim_switch = auto_aim_switch;

    // ④ 组装输入帧，交给流水线（记录提交时刻）
    AutoAimPipelineData::InitialData initial;
    initial.frame = std::move(img);
    initial.frame_timestamp = t;
    initial.source_timestamp_s = source_timestamp_s;
    initial.node_start_time = node_start_time;
    initial.performance_start_time = now;
    initial.bullet_velocity = static_cast<float>(state.bullet_velocity);
    initial.enemy_color = state.enemy_color;
    initial.pitch = static_cast<float>(state.pitch_rad);
    initial.yaw = static_cast<float>(state.yaw_rad);
    initial.total_yaw = static_cast<float>(state.total_yaw_rad);
    initial.roll = static_cast<float>(state.roll_rad);
    initial.ground_stable_point =
      cv::Point2f(500 + state.total_yaw_rad * fx, 500 + state.pitch_rad * fy);
    initial.auto_aim_switch = auto_aim_switch;
    initial.use_head_imu = state.use_head_imu;
    initial.mcu_yaw_online = state.mcu_yaw_online;
    initial.to_mcu_delta_yaw = static_cast<float>(state.to_mcu_delta_yaw);
    initial.to_mcu_delta_pitch = static_cast<float>(state.to_mcu_delta_pitch);
    pipeline.addFrame(std::move(initial));

    // ⑤ 取结果并下发
    auto result = pipeline.tryPopResult(std::chrono::steady_clock::now());
    if (result.valid) {
      result_fps.tick();  // 流水线完成一帧（有效结果）

      io::GimbalCommand cmd;
      cmd.yaw_torque_only_mode = torque_yaw_torque_only_mode;
      cmd.integral_enable = torque_integral_enable;
      if (result.valid_data.should_send_reset) {
        cmd.auto_aim_enable = false;
        if (no_target_hold_position) {
          // 保持模式：不回到 0，继续下发最后一次瞄准角度（fire 保持 false）
          cmd.pitch = last_send_pitch;
          cmd.yaw = last_send_yaw;
        }
      } else {
        cmd.auto_aim_enable = true;
        cmd.pitch = result.valid_data.mcu_command_pitch;
        cmd.yaw = result.valid_data.mcu_command_yaw;
        cmd.fire = result.valid_data.predictor_result.fire_flag;
        last_send_pitch = cmd.pitch;
        last_send_yaw = cmd.yaw;
      }
      gimbal.send(cmd);
      watchdog.feed_if_needed();

      // 保存最近一帧画面与命令，供实时面板使用（浅拷贝 + clone，避免改到 SHM 发布帧）
      if (!result.valid_data.visualizer_debug_frame.frame.empty()) {
        last_debug_frame = result.valid_data.visualizer_debug_frame.frame.clone();
      }
      last_cmd = cmd;
      last_has_target = !result.valid_data.should_send_reset;
      last_delta_pitch_deg =
        result.valid_data.predictor_result.command_delta_pitch / static_cast<float>(kDeg2Rad);
      last_delta_yaw_deg =
        result.valid_data.predictor_result.command_delta_yaw / static_cast<float>(kDeg2Rad);

      // ⑥ 每帧队列深度（debug）
      tools::logger()->debug(
        "armor_count: {} | Q[in:{} i0:{} i1:{} i2:{} out:{}]",
        result.valid_data.armor_count,
        result.always_valid_data.queue_input,
        result.always_valid_data.queue_inter0,
        result.always_valid_data.queue_inter1,
        result.always_valid_data.queue_inter2,
        result.always_valid_data.queue_output);
    }

    // ⑦ 滚动帧率：每 fps_log_interval 帧打一条
    loop_fps.tick();
    frame_count++;
    if (frame_count % fps_log_interval == 0) {
      tools::logger()->info(
        "[FPS] input {:.1f} fps ({:.2f} ms/frame) | pipeline {:.1f} fps (valid results) | in_q {}",
        loop_fps.fps(), loop_fps.avg_frame_time() * 1e3, result_fps.fps(),
        result.always_valid_data.queue_input);
    }

    // ⑧ 实时调试面板：视觉命令角 + torque 通道 MPC 输出（-headless 关闭窗口）
    if (!headless) {
      cv::Mat display;
      if (!last_debug_frame.empty()) {
        display = last_debug_frame.clone();
      } else {
        display = cv::Mat::zeros(720, 1280, CV_8UC3);
      }
      drawTorqueDebugPanel(
        display, last_cmd, gimbal.torqueDebugState(),
        last_has_target, last_delta_pitch_deg, last_delta_yaw_deg);
      cv::imshow("torque_debug", display);
      if (cv::waitKey(1) == 27) {  // ESC 退出
        tools::logger()->info("ESC pressed, exiting");
        break;
      }
    }

    // 帧率控制：与原 main_loop_func 一致，输入不超过 frame_rate
    std::this_thread::sleep_until(
      loop_start + std::chrono::microseconds(static_cast<int64_t>(1e6 / frame_rate)));
  }

  // 退出前补打一次阶段耗时总报告（若没到 report_interval）
  if (performance_monitor && performance_monitor->enabled()) performance_monitor->printStatistics();

  return 0;
}
