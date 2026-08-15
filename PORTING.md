# 搬运清单（transistor_rm2026 → sp_vision_25 架构，无 ROS2）

## 来源原则

- **壳**（架构、io 接口风格、tools、主程序）→ 来自 `sp_vision_25`
- **芯**（检测/跟踪/预测/解算/火控/通信协议）→ 来自 `transistor_rm2026_algorithm_visual_ws`

## ROS2 接缝替换规则

| 原写法 | 新写法 |
|---|---|
| `rclcpp::Node* node` 构造参数（仅用于日志） | 删除该参数，改用 `tools::logger()` |
| `RCLCPP_INFO/DEBUG/ERROR(node->get_logger(), "%.2f", x)` | `tools::logger()->info/debug/error("{}", x)` |
| `auto_aim::msg::*`（DebugArmor 等） | 普通 struct，字段不变 |
| `cv_bridge` + `sensor_msgs::msg::Image` | `cv::Mat` 直传 |
| `create_publisher` / ROS2 话题 | `tools::Plotter`（UDP json） |
| `std_msgs::msg::float32` | 删除（ba_solver 中实际未使用） |
| colcon/ament/rosidl | 普通 CMake（OpenCV/Eigen/spdlog/yaml-cpp/fmt/nlohmann_json/OpenVINO） |

## 文件清单

### A 组：整搬（0 处 ROS2 依赖，直接复制）

| 原路径（src/auto_aim/） | 去向 |
|---|---|
| include/2d_armor_detector/{LightBar,Params,UnwarpUtils}.h + src/2d_armor_detector/{ArmorDetector,UnwarpUtils}.cpp | tasks/auto_aim/2d_armor_detector/ |
| include/3d_processing/RestFrame.h + src/3d_processing/{RestFrame,BallisticSolver}.cpp | tasks/auto_aim/3d_processing/ |
| include/RP24_YOLO/OpenvinoInfer.h + src/RP24_YOLO/OpenvinoInfer.cpp | tasks/auto_aim/RP24_YOLO/ |
| include/camera/* + src/camera/cameraConnect.cpp | io/camera/ |
| include/communication/{CRC,SharedMemoryClassifier,HeadIMU,WatchdogClient}.h + src/communication/{CRC,SharedMemoryClassifier,HeadIMU,WatchdogClient}.cpp | io/communication/ |
| include/logger/* + src/logger/* | tools/（或 tasks 调试用） |
| include/other_input/* + src/other_input/* | io/other_input/ |
| include/predictor/{RotationMotionModel,PositionPredictor2D,PeriodicDataPredictor}.h + 对应 src | tasks/auto_aim/predictor/ |
| include/utils/* + src/utils/* | tools/ |
| include/visualizer/DataVisualizer.h + src/visualizer/DataVisualizer.cpp | tools/（接 plotter） |
| include/ba_solver/{utils,graph_optimizer}.hpp + src/ba_solver/{utils,graph_optimizer}.cpp | tasks/auto_aim/ba_solver/ |

### B 组：轻改（2~7 处 ROS2 行：删 node 参数 + 日志宏替换）

| 原路径 | 去向 | 改动点 |
|---|---|---|
| 2d_armor_detector/{Armor,ArmorClassifier,ArmorTracker,LightBarDetector}.h/.cpp | tasks/auto_aim/2d_armor_detector/ | 构造去 `node`，日志换 logger |
| 3d_processing/{ArmorSolver,BallisticSolver}.h + ArmorSolver.cpp | tasks/auto_aim/3d_processing/ | 同上 |
| RP24_YOLO/RP24_YOLO_Wrapper.h/.cpp | tasks/auto_aim/RP24_YOLO/ | 同上 |
| predictor/{AllPredictor,PredictorMain,PredictorSwitcher}.h/.cpp | tasks/auto_aim/predictor/ | 同上 |
| communication/Com.h/.cpp | io/communication/ | 同上 + 消息类型保持 struct |
| pipeline/AutoAimPipeline.h/.cpp | tasks/auto_aim/pipeline/ | 同上 |
| ba_solver/ba_solver.hpp/.cpp | tasks/auto_aim/ba_solver/ | 去 std_msgs include + 日志替换 |

### C 组：重写 / 删除

| 原路径 | 处理 |
|---|---|
| src/nodes/ArmorDetect_Node.cpp | 已改写为 users/infantry.cpp（无 ROS2 骨架，等 tasks 就位后接真管线） |
| msg/*.msg（DebugArmor 等） | 删除，改普通 struct（字段进 2d_armor_detector/Armor.h 等） |

## 搬运顺序

- [x] 0. 扫描接缝、本清单
- [x] 1. tools/（sp_vision_25 整搬）+ CMake 骨架（构建通过）
- [x] 2. io/camera（sp_vision_25 整搬，构建通过）
- [x] 3. io/communication + io/watchdog（transistor 去 ROS2，构建+冒烟测试通过）
- [x] 3.2 相机改回 transistor（GigE/USB，含视频/图片输入），删除 sp_vision_25 相机
- [x] 3.3 io/shm（SharedMemoryClassifier）+ io/ros2（可选 ROS 桥）
- [x] 4. tasks/auto_aim 全部模块去 ROS2（2d_armor_detector / 3d_processing / RP24_YOLO / ba_solver / predictor / pipeline），构建通过
- [x] 5. users/infantry.cpp 接 AutoAimPipeline 真管线，构建通过
- [x] 6. MODULES.md（各文件用途说明）
- [x] 6.1 assets/models + 测试视频搬入；configs/infantry.yaml 换为完整原配置
- [x] 6.2 视频回放全流程验证（YOLO→PnP→BA→火控 跑通）
- [x] 6.3 users/infantry_debug.cpp（滚动帧率 + 流水线结果速率 + 各阶段耗时报告）
- [ ] 7. 可视化接 tools::Plotter（原 publishVisualizerFrames）
- [ ] 8. 接 ROS 桥 / SHM 多进程联调
- [ ] 4. tasks/auto_aim/2d_armor_detector（A 组整搬 → B 组去 ROS2）
- [ ] 5. tasks/auto_aim/{3d_processing,RP24_YOLO,predictor,ba_solver,pipeline}
- [ ] 6. users/infantry.cpp 接真管线（先 pipeline 保行为，后拆直调）
- [ ] 7. 构建 + 用原视频跑通验证
