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
#include "io/gimbal_io.hpp"
#include "io/watchdog.hpp"
#include "pipeline/AutoAimPipeline.h"
#include "utils/PerformanceMonitor.h"
#include "utils/ThreadPool.h"
#include "tools/exiter.hpp"
#include "tools/cpu_affinity.hpp"
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
  }//这个main在解析key字符串得到合适的config_path

  // ---- 配置读取（提前到最前：cpu_pinning 需在创建任何线程前绑好主线程）----
  auto yaml = tools::load(config_path);
  auto config_file_ptr = std::make_shared<YAML::Node>(yaml);
  //得到YAML::Node的yaml,并且用config_file_ptr指向他
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
  io::Camera camera(config_path);        // 相机/视频/图片，阻塞取帧
  io::GimbalIo gimbal(config_path);      // 可切换下发模块：serial（原串口）/ torque（TorqueController）
  io::Watchdog watchdog(config_path);    // 看门狗

  // ---- 性能监控 + 全局线程池 ----
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
  if (thread_pool_size <= 0) {
    // 0=自动：绑核时按 E 核数量创建线程，避免 18 个 worker 挤在 8 个 E 核上超订
    thread_pool_size = static_cast<int>(tools::cpu_affinity::otherCpuCount());
    if (thread_pool_size <= 0) {
      thread_pool_size = static_cast<int>(std::thread::hardware_concurrency());
    }
  }
  ::utils::threadPool(thread_pool_size);

  // ---- tasks：算法流水线（四段：2D检测/分类 → 3D解算 → 预测/火控 → 可视化/日志）----
  AutoAimPipeline pipeline(config_file_ptr, workspace_path, node_start_time, performance_monitor);

  // ground_stable_point 需要相机内参焦距（原节点从 camera_matrix 提取）
  double fx = yaml["camera_matrix"][0][0].as<double>();
  double fy = yaml["camera_matrix"][1][1].as<double>();

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  double source_timestamp_s = 0.0;
  bool last_auto_aim_switch = true;
  float last_send_pitch = 0.0f;
  double last_send_yaw = 0.0;
  double frame_rate = yaml["frame_rate"] ? yaml["frame_rate"].as<double>() : 80.0;

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();

    // ① 取帧（阻塞到新帧，天然限帧，替代原 SYNC_CAMERA_FPS）
    camera.read(img, t, source_timestamp_s);
    if (img.empty()) continue;

    // ② 传感器状态（延迟对齐后）
    auto state = gimbal.stateAt(t);

    // ③ 自瞄开关 + HeadIMU 校准（原 processImage 的逻辑）
    bool auto_aim_switch = true;  // 原代码硬编码 true；以后可从电控读取
    if ((!last_auto_aim_switch && auto_aim_switch) &&
        state.use_head_imu && !state.mcu_yaw_online) {
      gimbal.recalibrateHeadImu();
    }//开启自瞄的瞬间，如果依赖 HeadIMU，就以此刻系统已经认可的 yaw 为基准，重新对齐 HeadIMU，并计算新的 offset，让重新校准后的 IMU 仍然对应这个 yaw。
    last_auto_aim_switch = auto_aim_switch;

    // ④ 组装输入帧，交给流水线
    AutoAimPipelineData::InitialData initial;
    initial.frame = std::move(img);  // 零拷贝移交，下一帧由 camera.read 重新填
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

    // ⑤ 取结果并下发（原 tryPopResult + sendData）
    auto result = pipeline.tryPopResult(std::chrono::steady_clock::now());
    if (!result.valid) continue;

    // 统一下发命令：serial 通道只用 pitch/yaw/fire；torque 通道额外用 auto_aim_enable。
    // reset 时 torque 通道把 auto_aim_enable 置 false（协议里与旧 reset 相反）。
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
