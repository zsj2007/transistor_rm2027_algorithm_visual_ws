// users/infantry_debug.cpp — 步兵调试版主程序
//
// 与 infantry.cpp 相同的管线，额外输出：
//   1. 滚动帧率（FrameRateCounter，主循环吞吐，窗口默认 60 帧）
//   2. 流水线结果速率（真正处理完成的帧数/秒）
//   3. 每 report_interval 帧自动打印各阶段耗时（PerformanceMonitor，stdout 报告）
//
// 用法：./build/infantry_debug configs/infantry_video.yaml

#include <chrono>
#include <filesystem>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/communication.hpp"
#include "io/watchdog.hpp"
#include "pipeline/AutoAimPipeline.h"
#include "utils/FrameRateCounter.h"
#include "utils/PerformanceMonitor.h"
#include "utils/ThreadPool.h"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace fs = std::filesystem;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/infantry_video.yaml | 位置参数，yaml配置文件路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  // ---- io：硬件抽象层（构造即初始化）----
  io::Camera camera(config_path);
  io::Communication comm(config_path);
  io::Watchdog watchdog(config_path);

  // ---- 配置 + 性能监控 + 全局线程池 ----
  auto yaml = tools::load(config_path);
  auto config_file_ptr = std::make_shared<YAML::Node>(yaml);
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
  bool last_auto_aim_switch = true;
  double frame_rate = yaml["frame_rate"] ? yaml["frame_rate"].as<double>() : 80.0;

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();

    // ① 取帧（阻塞到新帧）
    camera.read(img, t);
    if (img.empty()) continue;

    // ② 传感器状态（延迟对齐后）
    auto state = comm.state_at(t);

    // ③ 自瞄开关 + HeadIMU 校准
    bool auto_aim_switch = true;
    if ((!last_auto_aim_switch && auto_aim_switch) &&
        state.use_head_imu && !state.mcu_yaw_online) {
      comm.recalibrate_head_imu();
    }
    last_auto_aim_switch = auto_aim_switch;

    // ④ 组装输入帧，交给流水线（记录提交时刻）
    AutoAimPipelineData::InitialData initial;
    initial.frame = std::move(img);
    initial.frame_timestamp = t;
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

      if (result.valid_data.should_send_reset) {
        comm.send(0.0f, 0.0f, false);
      } else {
        comm.send(
          result.valid_data.mcu_command_pitch,
          result.valid_data.mcu_command_yaw,
          result.valid_data.predictor_result.fire_flag);
      }
      watchdog.feed_if_needed();

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

    // 帧率控制：与原 main_loop_func 一致，输入不超过 frame_rate
    std::this_thread::sleep_until(
      loop_start + std::chrono::microseconds(static_cast<int64_t>(1e6 / frame_rate)));
  }

  // 退出前补打一次阶段耗时总报告（若没到 report_interval）
  if (performance_monitor && performance_monitor->enabled()) performance_monitor->printStatistics();

  return 0;
}
