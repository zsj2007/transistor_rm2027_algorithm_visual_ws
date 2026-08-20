# 模块说明（MODULES.md）

本仓库 = **sp_vision_25 的壳（架构/工具/IO 接口）+ transistor_rm2026 的芯（算法/通信协议）**，无 ROS2 依赖（ROS 桥可选保留）。
来源标注：**S25** = sp_vision_25 搬运，**TR** = transistor_rm2026_algorithm_visual_ws 搬运。

## 架构

```
configs/  每台机器人的 YAML 配置
io/       硬件抽象层（输入源、通信、看门狗、共享内存、ROS 桥）
tasks/    功能层（检测/跟踪/解算/预测/火控，全部来自 TR）
tools/    万能工具（日志/绘图/退出/数学，全部来自 S25）
users/    应用层（步兵主程序）
```

数据流：`io::Camera → io::GimbalIo(state) → AutoAimPipeline → io::GimbalIo.send() → io::Watchdog`

## configs/

| 文件 | 说明 |
|---|---|
| infantry.yaml | 步兵配置（TR 的 src/shared_files/config.yaml 完整迁移），字段名与原项目一致 |
| infantry_video.yaml | 视频回放调试配置（USE_VIDEO: true），跑 assets/InputVideo/infantry_blue.mp4 |

## assets/（运行资源）

| 内容 | 说明 |
|---|---|
| models/ | YOLO 模型（yolov5_tongji.xml/.bin、model_rm2026_openvino、armor-oneclass-yolo11n 等），TR 原 src/shared_files/models 整搬 |
| InputVideo/ | 测试视频（infantry_blue.mp4），供 video 模式调试 |

## io/（硬件抽象层）

| 文件 | 来源 | 用途 |
|---|---|---|
| camera.hpp / camera.cpp | TR+S25 混合 | **输入源统一入口**：按 config 选 视频/图片/相机，对外 `read(img,t)` 阻塞取帧；持有原 TR 的全局帧交接区（g_image/g_mutex/g_bExit/image_used） |
| camera/Camera.h + cameraConnect.cpp | TR | GigE/USB 相机（海康 SDK），自管重连线程+取流线程 |
| camera/lib/ | /opt/MVS | 海康 MvCameraControl SDK |
| camera.hpp 里的 `::Camera` 包装 | TR | 原节点逻辑：曝光/增益设置、start/stop |
| communication.hpp / communication.cpp | TR 改写 | **串口+HeadIMU 统一入口**：收编原节点的 SerialCommunicationClass、HeadIMUSerialCommunicationClass、serial_infos_delay_ 延迟对齐、云台圈数累计、校准逻辑；`state_at(t)` 返回延迟对齐后的 `io::State`，`send(pitch,yaw,fire)` 下发 |
| communication/Com.h / Com.cpp | TR 改写 | 串口协议：115200 8N1，帧头 0x42 0x52，CRC8；解析 MCUDataFrame（弹速/云台角/颜色/转速），断线重连（udev 识别 USB 串口） |
| communication/HeadIMU.h / .cpp | TR | HeadIMU 串口通信：欧拉角/加速度/陀螺仪数据 |
| communication/CRC.h / CRC.cpp | TR | CRC8 校验 |
| communication/WatchdogClient.h / .cpp | TR | Unix 域套接字看门狗客户端 |
| watchdog.hpp / watchdog.cpp | TR 包装 | **看门狗封装**：按 config 周期喂狗，失败自动降级 |
| gimbal_io.hpp / gimbal_io.cpp | 新增 | **可切换下发模块**：`command_channel: serial | torque` 双通道。serial = 原 `io::Communication` 角度协议（0x42 0x52 0xCD）；torque = TorqueController 的 `RobotController`（MCU/IMU 串口 + 融合 + yaw MPC + 后台 100Hz 发送，由 `~/TorqueController` 源码编入，需 Ceres）。torque 模式下传感器状态改由 `RobotController::getState()` 映射，不再占用旧串口 |
| other_input/VideoInput.h / .cpp | TR | 视频文件输入（调试用），写入全局帧区 |
| other_input/ImagesInput.h / .cpp | TR | 图片集输入（调试用），写入全局帧区 |
| shm/SharedMemoryClassifier.h / .cpp | TR | **多进程共享内存通信窗口**：SHM + POSIX 信号量，把裁剪图发给 Python 分类进程并取回结果（CLASSIFIER_SHM_KEY 从 config 读） |
| ros2/（publish2nav、subscribe2nav、ros2.hpp/cpp） | S25 | **可选 ROS 桥**：检测到 ROS2 环境才编译，发布/订阅导航数据 |

## tasks/auto_aim/（功能层，全部 TR）

### 2d_armor_detector（灯条/装甲板检测与分类）

| 文件 | 用途 |
|---|---|
| Params.h | 检测参数结构体（阈值、颜色等） |
| LightBar.h | 灯条结构体（RotatedRect + 长度/角度） |
| LightBarDetector.h / .cpp | 灯条检测：颜色通道差分、过滤、ROI 扩展 |
| Armor.h / Armor.cpp | 装甲板结构体：角点计算、ROI 展开、物理尺寸常量（小/大装甲板、灯条高度） |
| ArmorDetector.h / .cpp | 灯条配对 → 装甲板（角度/高度/距离/置信度过滤） |
| ArmorClassifier.h / .cpp | 装甲板分类：ROI 预处理 → SHM 送给 Python 分类器 → 跟踪器更新 |
| ArmorTracker.h / .cpp | 装甲板跟踪：多目标状态机、位置预测（PositionPredictor2D）、丢失/恢复 |
| UnwarpUtils.h / .cpp | ROI 展平/透视变换工具 |

### 3d_processing（PnP 解算与弹道）

| 文件 | 用途 |
|---|---|
| ArmorSolver.h / .cpp | solvePnP 求装甲板 3D 姿态/距离；内参、畸变、枪口偏移(delta_x/y/z)从 config 读；3D↔像素投影 |
| BallisticSolver.h / .cpp | 弹道解算：空气阻力模型、RK4/欧拉仿真、最近点/命中高度迭代求 pitch |
| RestFrame.h / .cpp | 静止坐标系（相机系↔世界系转换） |

### RP24_YOLO（YOLO 检测）

| 文件 | 用途 |
|---|---|
| OpenvinoInfer.h / .cpp | OpenVINO 推理封装：多 InferRequest 并发、letterbox、关键点解码 |
| RP24_YOLO_Wrapper.h / .cpp | YOLO 异步流水线：提交帧/取结果、类别映射（同济9类 / FasterNet17类）、灯条关键点 → Armor |

### ba_solver（图优化）

| 文件 | 用途 |
|---|---|
| ba_solver.hpp / .cpp | g2o 光束平差：优化装甲板 yaw 姿态，减小投影误差 |
| graph_optimizer.hpp / .cpp | g2o 图优化器/顶点/边封装 |
| utils.hpp / .cpp | 旋转矩阵/欧拉角转换等数学工具 |

### predictor（运动预测）

| 文件 | 用途 |
|---|---|
| PeriodicDataPredictor.h / .cpp | 周期性数据预测（小陀螺旋转） |
| RotationMotionModel.h / .cpp | 旋转运动模型 |
| PositionPredictor2D.h / .cpp | 2D 位置预测：线性/二次/傅里叶拟合 |
| PredictorSwitcher.h / .cpp | 预测器切换（None/Periodic/Position2D…） |
| AllPredictor.h / .cpp | 单类装甲板的完整预测：解算→静止系→预测→弹道→火控决策 |
| PredictorMain.h / .cpp | 预测总管：按类别维护多个 AllPredictor、PID 云台角积分、目标切换 |

### pipeline（流水线调度）

| 文件 | 用途 |
|---|---|
| AutoAimPipeline.h / .cpp | **四段流水线**：Stage1 2D检测/分类/YOLO → Stage2 3D解算 → Stage3 预测/火控 → Stage4 可视化/日志；线程池驱动、队列限流、按序取结果 |

### 支撑模块

| 文件 | 用途 |
|---|---|
| utils/ThreadPool.h | 通用线程池（任务提交/完成通知） |
| utils/PerformanceMonitor.h / .cpp | 各阶段耗时统计 |
| utils/FrameRateCounter.h / .cpp | 帧率统计 |
| utils/VisualizerConfig.h | 可视化开关配置（从 yaml 读） |
| utils/DataProcessFuncs.h / .cpp | 数据处理工具 |
| utils/SimpleDataFilter.h / .cpp | 简单数据滤波 |
| utils/PeriodFunctions.h / .cpp | 周期函数工具 |
| visualizer/DataVisualizer.h / .cpp | 画图/示波器（原 ROS2 发布版后续接 tools::Plotter） |
| logger/MkvWriter.h / .cpp | ffmpeg 录 MKV 视频 |
| logger/TwoVideoLogger.h / .cpp | 双路视频录制（原始画面+处理结果） |

## tools/（万能工具，全部 S25）

| 文件 | 用途 |
|---|---|
| logger.hpp / .cpp | spdlog 封装，全局 `tools::logger()`（替代原 RCLCPP_*） |
| exiter.hpp / .cpp | Ctrl+C 退出标志 |
| yaml.hpp | YAML 加载/读取（缺字段直接报错退出） |
| plotter.hpp / .cpp | UDP JSON 绘图客户端（后续接可视化） |
| recorder.hpp / .cpp | OpenCV VideoWriter 录制 |
| math_tools.hpp / .cpp | 欧拉角/旋转矩阵/坐标变换 |
| img_tools.hpp / .cpp | 画点/画轮廓/画文字辅助 |
| pid.hpp / .cpp | PID 控制器 |
| extended_kalman_filter.hpp / .cpp | EKF |
| trajectory.hpp / .cpp | 弹道轨迹工具 |
| ransac_sine_fitter.hpp / .cpp | RANSAC 正弦拟合 |
| crc.hpp / .cpp | CRC 校验 |
| thread_safe_queue.hpp | 线程安全队列 |

## users/

| 文件 | 用途 |
|---|---|
| infantry.cpp | **步兵主程序**：CLI 配置路径 → 构造 io（相机/通信/看门狗）+ 线程池 + AutoAimPipeline → 主循环（取帧→状态→喂流水线→取结果→下发→喂狗）。原 ArmorDetect_Node 的 main+processImage 合并后的无 ROS2 版 |
| infantry_debug.cpp | **调试版主程序**：同管线 + 每 30 帧打印滚动帧率（输入 fps / 流水线结果 fps / 队列深度），每 report_interval 帧自动打印各阶段耗时报告 |

## 帧率与耗时怎么看

- 滚动帧率：`[FPS] input xx.x fps | pipeline xx.x fps (valid results) | in_q n`（infantry_debug 每 30 帧一条）
- 各阶段耗时：PerformanceMonitor 每 `performance_monitor_report_interval`（默认 90）帧自动打印 `Auto Aim Performance` 报告：
  `stage1_yolo_latency / yolo_preprocess / yolo_infer / yolo_infer_wait / yolo_postprocess / stage2_3d_solve_transform / stage3_predict_command / stage4_visualize_log`，含 avg/min/max。
- 注意：报告里的 `FPS` = 1/平均帧延迟（误导），真实吞吐看日志行里的 `pipeline xx.x fps`。

## 构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/infantry configs/infantry.yaml
```

依赖：OpenCV、fmt、Eigen3、spdlog、yaml-cpp、nlohmann_json、g2o、Sophus、TBB、OpenVINO、ffmpeg、海康 MVS SDK、udev。
