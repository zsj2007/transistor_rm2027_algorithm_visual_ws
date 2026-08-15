// users/infantry.cpp — 步兵主程序（无 ROS2，sp_vision_25 架构）
//
// 由 transistor_rm2026_algorithm_visual_ws 的 ArmorDetect_Node.cpp 改造：
//   - 去掉 rclcpp / cv_bridge / ROS2 话题（可视化后续接 tools::Plotter）
//   - 初始化全部放在各对象构造函数，主循环由 main() 驱动
//   - 输入源 io::Camera、传感器 io::Communication、看门狗 io::Watchdog
//   - 算法走 tasks::AutoAimPipeline（检测→跟踪→分类→解算→预测→火控）

#include <chrono>
#include <filesystem>
#include <memory>
#include <thread>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/communication.hpp"
#include "io/watchdog.hpp"
#include "pipeline/AutoAimPipeline.h"
#include "utils/PerformanceMonitor.h"
#include "utils/ThreadPool.h"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace fs = std::filesystem;

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | configs/infantry.yaml | 位置参数，yaml配置文件路径}";

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
  io::Camera camera(config_path);        // 相机/视频/图片，阻塞取帧
  io::Communication comm(config_path);   // 串口 MCU + HeadIMU + 延迟对齐
  io::Watchdog watchdog(config_path);    // 看门狗

  // ---- 配置 + 性能监控 + 全局线程池（原节点初始化顺序）----
  auto yaml = tools::load(config_path);
  auto config_file_ptr = std::make_shared<YAML::Node>(yaml);
  auto node_start_time = std::chrono::steady_clock::now();
  fs::path workspace_path = fs::path(config_path).parent_path().parent_path();  // 项目根

  bool perf_enabled = yaml["performance_monitor_enabled"]
                        ? yaml["performance_monitor_enabled"].as<bool>()
                        : true;
  size_t perf_interval = yaml["performance_monitor_report_interval"]
                           ? yaml["performance_monitor_report_interval"].as<size_t>()
                           : 90;
  auto performance_monitor = std::make_shared<PerformanceMonitor>(perf_enabled, perf_interval);

  int thread_pool_size = yaml["thread_pool_size"] ? yaml["thread_pool_size"].as<int>() : 0;
  ::utils::threadPool(thread_pool_size);

  // ---- tasks：算法流水线（四段：2D检测/分类 → 3D解算 → 预测/火控 → 可视化/日志）----
  AutoAimPipeline pipeline(config_file_ptr, workspace_path, node_start_time, performance_monitor);

  // ground_stable_point 需要相机内参焦距（原节点从 camera_matrix 提取）
  double fx = yaml["camera_matrix"][0][0].as<double>();
  double fy = yaml["camera_matrix"][1][1].as<double>();

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  bool last_auto_aim_switch = true;
  double frame_rate = yaml["frame_rate"] ? yaml["frame_rate"].as<double>() : 80.0;

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();

    // ① 取帧（阻塞到新帧，天然限帧，替代原 SYNC_CAMERA_FPS）
    camera.read(img, t);
    if (img.empty()) continue;

    // ② 传感器状态（延迟对齐后）
    auto state = comm.state_at(t);

    // ③ 自瞄开关 + HeadIMU 校准（原 processImage 的逻辑）
    bool auto_aim_switch = true;  // 原代码硬编码 true；以后可从电控读取
    if ((!last_auto_aim_switch && auto_aim_switch) &&
        state.use_head_imu && !state.mcu_yaw_online) {
      comm.recalibrate_head_imu();
    }
    last_auto_aim_switch = auto_aim_switch;

    // ④ 组装输入帧，交给流水线
    AutoAimPipelineData::InitialData initial;
    initial.frame = std::move(img);  // 零拷贝移交，下一帧由 camera.read 重新填
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

    // ⑤ 取结果并下发（原 tryPopResult + sendData）
    auto result = pipeline.tryPopResult(std::chrono::steady_clock::now());
    if (!result.valid) continue;

    if (result.valid_data.should_send_reset) {
      comm.send(0.0f, 0.0f, false);
    } else {
      comm.send(
        result.valid_data.mcu_command_pitch,
        result.valid_data.mcu_command_yaw,
        result.valid_data.predictor_result.fire_flag);
    }
    watchdog.feed_if_needed();

    // ⑥ 可视化（原 publishVisualizerFrames）：后续接 tools::Plotter / DataVisualizer
    // plotter.show(result.valid_data.visualizer_debug_frame.frame, ...);

    tools::logger()->debug(
      "armor_count: {} | Q[in:{} i0:{} i1:{} i2:{} out:{}]",
      result.valid_data.armor_count,
      result.always_valid_data.queue_input,
      result.always_valid_data.queue_inter0,
      result.always_valid_data.queue_inter1,
      result.always_valid_data.queue_inter2,
      result.always_valid_data.queue_output);

    // 帧率控制：与原 main_loop_func 一致，输入不超过 frame_rate
    std::this_thread::sleep_until(
      loop_start + std::chrono::microseconds(static_cast<int64_t>(1e6 / frame_rate)));
  }

  return 0;
}
