# transistor_rm2027_algorithm_visual_ws

2027 赛季步兵自瞄算法仓库：无 ROS2，纯 C++17 / CMake。，调试可视化通过**共享内存**（System V SHM + POSIX 信号量）在算法进程和独立可视化进程之间传输，替代 2026 仓库里基于 ROS 话题的 `auto_aim_visualizer`。

当前主线是**新架构 + 新预测器**：

- **新架构**：帧输入统一走 `FramePacket`（像素 + 源帧时间戳 `source_timestamp_s` + `frame_id`），流水线四段异步化，预测器按源帧时间推进；
- **新预测器**：`TargetManager`（持久目标生命周期管理）+ `SuperPowerEKF`（基于 EKF 的旋转目标预测），RMM 已从构建中移除；
- **绑核**：每台机器的 CPU 拓扑不同，必须按第 3.4 节自己配置，禁止照抄。



## 快速开始（Ubuntu 22.04 x86_64）

```bash
# 0. 一键安装/验证依赖并编译（推荐；等效于下面的 1+2）
./setup.sh

# 1. 安装依赖（详见下文）
# 2. 编译
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 3. 无硬件自检：视频回放
./build/infantry_debug configs/infantry_video.yaml

# 4. 另开一个终端，启动可视化（读同一份配置）
./build/visualizer configs/infantry_video.yaml
```

## 目录结构

```
configs/   每台机器人的 YAML 配置（infantry.yaml 实车 / infantry_video.yaml 视频回放）
assets/    运行资源：YOLO 模型（已入库）、测试视频（已入库）
io/        硬件抽象层：相机/串口/看门狗/共享内存；帧输入统一发布到 FramePacket（像素+源时间戳+帧号）
tasks/     算法功能层：检测(YOLO)/跟踪/分类/解算/预测/火控/Stage4 可视化
  auto_aim/EKF/         SuperPower EKF 预测系统（SuperPowerEKF/Target/Tracker/Predictor）
  auto_aim/predictor/   TargetManager + AllPredictor/PredictorMain（新 step 接口）
TorqueController/ 力矩控制仓库（vendored，torque 通道用；缺 Ceres 时仅 serial 可用）
tools/     通用工具：日志（spdlog）、数学、CPU 绑核等
users/     应用层：infantry、infantry_debug、visualizer 三个可执行文件
scripts/   运维脚本：cpu_topology.sh（识别每台机器 P/E 核并给出绑核建议）
```

数据流：`io::Camera → io::GimbalIo(state) → AutoAimPipeline（4 段流水线）→ io::GimbalIo.send() → io::Watchdog`，可视化数据在流水线 Stage4 写入共享内存。

## 架构与数据流（当前状态）

四段流水线：`Stage1 检测(YOLO) → Stage2 3D解算 → Stage3 预测/火控 → Stage4 可视化/日志`。

**1. 帧时间戳（新架构核心）**

相机 / 视频 / 图片输入统一把每帧发布到全局 `FramePacket`（`image` + `timestamp_s` + `frame_id`），`read()` 在同一个临界区里取齐像素和时间，不再出现“取帧前打时间戳导致时间错配”。流水线的 `InitialData.source_timestamp_s` 会传给 `PredictorMain::step`，EKF 用它算 `dt`——**视频回放时即使流水线跑得比实时快，dt 仍按源帧率推进**，预测行为与实车一致。

**2. 新预测器（SuperPower EKF）**

`PredictorMain::step` 的旧签名（`predictor_type` 自动切换）已改为传 `source_timestamp_s`，内部走：

- `TargetManager`：维护持久物理目标的完整生命周期（`DETECTING / TRACKING / TEMP_LOST / LOST / SWITCHING`），配置段 `target_manager:`（`min_detect_frames`、`max_temp_lost_frames`、`outpost_max_temp_lost_frames`、`enable_priority_switch`）；
- `SuperPowerEKF`：EKF 状态估计 + 旋转目标预测（适配四装甲板），配置段 `superpower_ekf:`（`min_detect_count`、`max_temp_lost_count`、`max_dt_s`、`initial_radius_m`、`armor_num`）。

注意：`RotationMotionModel` 已从构建中移除，不再参与预测。visualizer 增加了 `SuperPowerEKF` 预测器类型和目标状态面板。

**3. 性能相关配置**

- 每台机器先跑 `./scripts/cpu_topology.sh` 识别 P/E 核，按第 3.4 节配置 `cpu_pinning`；
- `frame_rate` 应 ≤ 流水线处理能力，`in_q` 稳定 0~1 为健康（见第 4 节）；
- 推理慢/掉帧优先查：绑核是否生效、是否插电、`yolo_infer` 是否热降频。

## 1. 依赖安装

### 1.1 系统依赖（apt）

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
  libopencv-dev libfmt-dev libeigen3-dev libspdlog-dev libyaml-cpp-dev \
  nlohmann-json3-dev libtbb-dev libudev-dev \
  libsuitesparse-dev \
  ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev
```

串口权限（操作 `/dev/ttyACM*` 需要 `dialout` 组，**改完要重新登录**）：

```bash
sudo usermod -aG dialout $USER
```

### 1.2 Sophus 和 g2o（源码编译）

```bash
# Sophus（依赖 Eigen3 + fmt）
git clone https://github.com/strasdat/Sophus.git
cd Sophus && git checkout 1.22.10
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SOPHUS_EXAMPLES=OFF -DBUILD_SOPHUS_TESTS=OFF
make -j$(nproc) && sudo make install
cd ../..

# g2o（依赖 Eigen3 + suitesparse）
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF \
  -DG2O_BUILD_LINKED_APPS=OFF -DG2O_BUILD_PLUGINS_DEFAULT=OFF
make -j$(nproc) && sudo make install
cd ../..

sudo ldconfig
```

### 1.3 OpenVINO（CMake 通过 pkg-config 查找）

方法 A（推荐）：去 Intel 官网下载 OpenVINO 2024 Runtime 的 `.deb`（Ubuntu 22.04 amd64）：

```bash
sudo apt install -y ./openvino_2024.*_ubuntu22_amd64.deb
# 让 pkg-config 找到 openvino.pc（路径按实际安装版本改）
export PKG_CONFIG_PATH=/opt/intel/openvino_2024*/runtime/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/intel/openvino_2024*/runtime/lib:$LD_LIBRARY_PATH
pkg-config --modversion openvino   # 应输出版本号，如 2024.6.0
```

方法 B（快速验证）：`pip install openvino==2024.6.0`，再把其 `openvino.pc` 所在目录加入 `PKG_CONFIG_PATH`。

### 1.4 可选依赖

- **ROS2 桥**（`io/ros2/`）：检测到 ROS2 环境才编译，没有也能正常构建（CMake 会提示 `ROS2 not found, skipping ROS2 bridge`）。
- **TorqueController 力矩下发通道**（`command_channel: torque`）：源码已 vendor 在仓库内 `TorqueController/`，**只需系统装好 Ceres**；Ceres 缺失时仅编译 serial 串口通道（默认即 serial）。CMake 优先用仓库内目录，找不到再回退 `$HOME/TorqueController`。
- **相机 SDK**：海康 `libMvCameraControl.so` 已随仓库入库（`io/camera/lib/`），无需另外安装。
- **模型与测试视频**：`assets/models/`、`assets/InputVideo/` 已入库，无需下载。

## 2. 编译

```bash
cd ~/transistor_rm2027_algorithm_visual_ws
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # 默认就是 Release，可省略
cmake --build build -j$(nproc)
```

生成三个可执行文件：

| 目标 | 用途 |
|---|---|
| `build/infantry` | 实车主程序（相机 + 串口/力矩下发） |
| `build/infantry_debug` | 调试版：多打印滚动帧率 + 每 90 帧各阶段耗时报告 |
| `build/visualizer` | 独立可视化进程（共享内存读取，无 ROS） |

## 3. 运行

所有命令都在仓库根目录执行（模型/视频路径是相对根目录的）。

### 3.1 实车模式（GigE 相机 + 串口）

```bash
./build/infantry configs/infantry.yaml
```

相机网络配置见 [DEPLOY.md](DEPLOY.md)（上位机网卡固定 `192.168.10.25/24`，相机 `192.168.10.10`）。

### 3.2 视频回放调试（无硬件自检）

```bash
./build/infantry_debug configs/infantry_video.yaml
```

`infantry_video.yaml` 使用 `assets/InputVideo/infantry_blue.mp4` 循环播放，串口不可用时自动降级继续跑。

### 3.3 可视化（共享内存，无 ROS）

```bash
# 先跑算法（任意配置），再另开终端：
./build/visualizer configs/infantry_video.yaml
```

可视化进程读取算法进程 Stage4 写入共享内存的调试帧，画四个窗口：

- **Armor Detection**：主检测画面（灯条/装甲板/解算结果/云台坐标系/状态文字）
- **Yaw Visualizer**：云台 yaw 曲线（目标 vs 当前）
- **RMM visualize**：旋转运动模型平面可视化
- **Common Debug Oscilloscope**：通用调试示波器

常用选项：

```bash
./build/visualizer configs/infantry_video.yaml -headless    # 无窗口，只打印接收统计（无显示环境调试）
./build/visualizer configs/infantry_video.yaml -frames=100  # 收满 100 帧自动退出
```

窗口内按 `ESC` / `q` 退出，终端里按 `Ctrl+C` / `Ctrl+Z` 也会优雅退出（`Ctrl+Z` 已从默认的挂起改为退出）。相关配置项（两个 config 都有）：

| 配置 | 含义 |
|---|---|
| `visualizer.enable` | 总开关（算法侧 + 可视化侧都读） |
| `visualizer.publish_topics` | 算法侧是否把调试帧写入共享内存（替代 2026 的 ROS 话题） |
| `visualizer.shm_key` | 共享内存 key，默认 `0x0251`，两个进程必须一致 |
| `visualizer.show_windows` | 可视化进程是否开窗口 |
| `visualizer.draw.*` | 每个窗口/图层单独开关 |

可视化进程没启动时，算法侧会自动跳过发布（几乎零开销）；启动后每帧多约 0.5~1ms（Stage4 共享内存拷贝）。不需要调试画面时把 `publish_topics: false` 即可完全关掉。

### 3.4 CPU 绑核（每台机器必须自己配，禁止照抄）

`configs/*.yaml` 里的 `cpu_pinning:` 是按**本机 CPU 拓扑**写的，换机器必须重新配置，否则会踩两个坑：

1. **P/E 核识别错误**：不要按“频率最高那组= P 核”猜，Intel 混合架构里 P 核的最大频率可能不统一（如 i9-12900HK 是 4.9/5.0GHz 混着），正确依据是**超线程**——P 核一个物理核有 2 个逻辑核、E 核没有。跑仓库自带的脚本自动识别：

   ```bash
   ./scripts/cpu_topology.sh
   ```

   它会打印每核最大频率、超线程配对，并给出 `other_cores` / `yolo_core_type` / `RP24_YOLO_infer_threads` 建议。

2. **`enabled: true` 会锁死推理线程**：`enabled: true` 把主线程绑到 `other_cores`（E 核），之后创建的 OpenVINO 推理线程会继承这个亲和掩码，此时 `yolo_core_type: "pcore"` **无法突破**，推理会实际跑到 E 核上、速度暴跌。当前代码下正确的用法是：

   ```yaml
   cpu_pinning:
     enabled: false        # 别绑主线程，让 OpenVINO 自己能迁移
     yolo_core_type: "pcore"
     yolo_enable_cpu_pinning: true
   RP24_YOLO_infer_threads : <物理P核数>   # HT off；开HT 可试 <物理P核数×2>
   ```

   各字段含义：`other_cores` = 主线程/流水线等非推理线程的核；`yolo_core_type` = 推理线程限定 pcore/ecore/any；`yolo_enable_hyper_threading` = 是否用超线程逻辑核；`yolo_cores` + `yolo_pool_threads` = 给 YOLO 预处理/后处理单独建线程池（建议放 E 核，别和推理抢 P 核）。

   笔记本上跑满 3 分钟后 `yolo_infer` 如果从低值缓慢爬升，通常是热/功耗降频：先确认插电 + `sudo powerprofilesctl set performance`，再考虑换 `ecore` 或降低 `frame_rate` 匹配处理能力（`in_q` 稳定 0~1 为健康）。

**操作步骤速查（每台新机器都做一遍）：**

```bash
# 1. 识别本机拓扑，抄建议值
./scripts/cpu_topology.sh

# 2. 填进 configs/*.yaml 的 cpu_pinning: 段
#    enabled: false + yolo_core_type: "pcore" + yolo_enable_cpu_pinning: true
#    RP24_YOLO_infer_threads = 物理 P 核数（开 HT 可试 ×2）

# 3. 启动后核对生效 + 跑满 3 分钟验收
./build/infantry_debug configs/infantry_video.yaml
#    看启动日志 main thread allowed cpus（应为全核）与 yolo pool 行
#    看 [FPS] pipeline 是否稳定、in_q 是否 0~1、yolo_infer 是否不爬升
```

## 4. frame rate / algorithm rate / 日志 FPS 到底是什么

这三个数各自测量的是**不同环节的速率**，别拿它们直接对等：

| 数值 | 谁测的 | 含义 |
|---|---|---|
| 日志 `[FPS] input` | 算法主循环 | 每秒从相机/视频读到多少帧，受视频帧率或 `frame_rate` 配置限制 |
| 日志 `[FPS] pipeline` | 算法主循环 | 流水线真正处理完（4 段跑完）并被取回的帧数/秒，即算法实际吞吐 |
| 窗口 `algorithm rate`（黄） | 算法 Stage4 | 每帧写入共享内存的发布速率，由发布侧滚动 1s 窗口实测，**和日志 pipeline FPS 一致** |
| 窗口 `frame rate`（绿） | 可视化进程 | 可视化进程自己每秒实际渲染并显示了多少帧 |

为什么 `frame rate` 会明显低于另外两个：可视化进程每帧要拷贝约 14MB 快照（原始 1280×1024 图 + RMM + 示波器）、画全部标注、再刷新 4 个窗口，渲染不过来时**中间帧直接丢弃、永远显示最新一帧**（last-writer-wins），所以它天然 ≤ 算法发布速率。这是设计如此，不是算法变慢或丢数据。本机实测（pipeline 约 70fps）：headless 模式约 69fps（跟得上），开窗口约 48fps——瓶颈在窗口刷新，远程/虚拟显示（如 NoMachine）下差距更大。

为什么日志 `input` 和 `pipeline` 也有差异：`input` 是取帧上限，`pipeline` 是算法真实处理能力（含 YOLO 推理等），处理不过来时 pipeline 低于 input，此时 `in_q`（输入队列深度）会持续大于 3~4。

注意：窗口里的 `algorithm rate` 是发布侧实测值，不会因为可视化渲染慢而下降；想确认算法没变慢就看它，想确认画面实时性就看 `frame rate`。

想提高 `frame rate`：把不看的窗口关掉（`visualizer.draw.rmm`、`visualizer.draw.common_debug_oscilloscope` 等设 `false`），并在本地有线显示上跑。

## 5. 日志怎么看

日志用 spdlog 输出，级别前缀 `[info]` / `[warning]` / `[error]`。

### 5.1 滚动帧率（`[FPS]`，每 30 帧一条）

```text
[FPS] input 89.4 fps (11.19 ms/frame) | pipeline 82.5 fps (valid results) | in_q 2
```

| 字段 | 含义 |
|---|---|
| `input` | 主循环取帧吞吐，受视频帧率 / 相机帧率 / `frame_rate` 配置限制 |
| `pipeline` | 流水线真正处理完并取回结果的帧数/秒 |
| `in_q` | 输入队列深度。持续大于 3~4 说明下游处理不过来，帧在排队（延迟在累积） |

### 5.2 阶段耗时报告（`Auto Aim Performance`，每 90 帧一条）

```text
========== Auto Aim Performance ==========
stage1_yolo_latency: avg 53.9 ms [min 26.2, max 65.7]   # 2D 检测总耗时（YOLO 路径）
yolo_preprocess: avg 0.86 ms                             # 预处理（letterbox 等）
yolo_infer: avg 12.4 ms                                  # 推理本身
yolo_infer_wait: avg 38.6 ms                             # 推理排队等待（异步流水线）
yolo_postprocess: avg 0.15 ms                            # 后处理
stage2_3d_solve_transform: avg 0.47 ms                   # 3D 解算/坐标系变换
stage3_predict_command: avg 2.2 ms                       # 预测 + 火控解算
stage4_visualize_log: avg 0.95 ms                        # 可视化发布 + 视频录制
stage1_classify_track: avg 0.02 ms                       # 分类/跟踪
------------------------------------------
Total(from data init): 89.4 ms
FPS: 11.2
```

要点：

- `yolo_infer_wait` 明显偏大说明推理请求在排队（异步 YOLO 流水线），可调 `RP24_YOLO_infer_threads` / `RP24_YOLO_infer_streams`。
- 挂载可视化后 `stage4_visualize_log` 约 0.5~1ms（共享内存拷贝），没挂时约 0.02ms，属正常。
- 报告底部 `FPS` 是该 90 帧窗口的端到端吞吐，`input FPS` 和它差得远时，优先看上面的 stage 耗时找瓶颈。

### 5.3 常见提示

```text
[VisualizerShm] writer ready (publish debug frames via shared memory)   # 可视化发布通道就绪
Serial port not available, trying reconnect                             # 串口未插/无权限，自动重连（视频模式不影响）
[Watchdog] init failed, running without watchdog                        # 看门狗没起，程序独立运行
[visualizer] waiting for algorithm data...                              # 可视化进程已挂接，但算法还没发布数据
```

## 常见问题

- `pkg-config --modversion openvino` 报错 → 第 1.3 步没配对 `PKG_CONFIG_PATH`。
- 编译报找不到 g2o/Sophus → 第 1.2 步没装好，`sudo ldconfig` 后再 cmake。
- 相机连不上 → 检查网卡 IP、`ping 192.168.10.10`、网线/供电。
- 串口打不开 → 检查 `dialout` 组、`ls /dev/ttyACM*`，改完组要重新登录。
- 模型路径错误 → 确认从仓库根目录运行，`assets/models/` 完整。
- 可视化收不到数据 → 确认算法进程先启动（或两端 `shm_key` 一致）、`visualizer.publish_topics: true`。
