# TorqueController 下发通道切换说明

记录本次「可切换下发模块」的接入方式、两个通道的差异，以及目前已知的两个待处理问题
（yaw 目标 0.2s 延迟、`auto_aim_switch` 硬编码）。

## 1. 背景

原来 `infantry` 只有一条下发通道：`io::Communication` 走旧串口角度协议
（帧头 `0x42 0x52 0xCD`，下发 pitch/yaw 角度 + fire）。

本次新增了第二条通道：`~/TorqueController` 的 `RobotController` 一体化封装，
由它自己管理 MCU/IMU 串口、IMU 融合、yaw MPC 求解和后台 100Hz 发送。

两条通道通过 [io/gimbal_io.hpp](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/io/gimbal_io.hpp)
中的 `GimbalIo` 门面类切换，`infantry.cpp` 主循环只跟 `GimbalIo` 打交道。

## 2. 切换方法

### 2.1 改配置

编辑 [configs/infantry.yaml](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/configs/infantry.yaml:33)：

```yaml
# serial：原串口角度协议（默认，行为与改动前一致）
# torque：TorqueController MPC 力矩协议
command_channel : "serial"   # 改成 "torque" 即切换
```

torque 通道的 MPC 参数全部在 `torque_controller:` 段（同一文件第 36 行起），
默认值与 `~/TorqueController/src/control_demo.cpp` 一致。换车时记得改：

- `J / tau_c / b / tau_d`：辨识参数，来自
  `~/TorqueController/data/cars/<车名>/params/Identified_parameters.txt`；
- `send_pitch_scale / send_pitch_offset / recv_pitch_scale / recv_pitch_offset`：
  MCU 数据线性标定，来自
  `~/TorqueController/data/cars/<车名>/LinearParams.txt`。

### 2.2 构建

torque 通道需要 Ceres（MPC 求解）和 `~/TorqueController` 源码。`io/CMakeLists.txt`
在两者都满足时把 TorqueController 源码编进 `io` 库并定义
`IO_ENABLE_TORQUE_CONTROLLER`；缺任一则自动降级为仅 serial（配置写 torque 会报错退出）。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

配置阶段看到 `TorqueController channel enabled: ...` 即接入成功。

### 2.3 验证

启动日志会打印当前通道：

```
[info] GimbalIo channel: serial     # 或 torque
[info] TorqueGimbalSender ready (dt=0.01s N=20 ...)   # 仅 torque 通道
```

## 3. 两个通道的差异

| 项目 | serial | torque |
|---|---|---|
| 下发协议 | 旧角度协议 `0x42 0x52 0xCD`（pitch/yaw/fire） | TorqueController 力矩协议 `0x42 0x52 0x02`（含 yaw_torque） |
| 发送频率 | 主循环每帧 `send()` 一次 | 主循环 `set()` 只更新目标，后台线程固定 100Hz 发送 |
| 传感器状态 | `io::Communication`（MCU 串口 + HeadIMU + 延迟对齐） | `RobotController::getState()`（自带融合，`use_head_imu=false`） |
| 串口占用 | io::Communication 开 MCU/HeadIMU 串口 | RobotController 开 MCU/IMU 串口（两套不能同时开，故切换状态源） |
| HeadIMU 校准 | `recalibrateHeadImu()` 有效 | 空操作（融合在 TorqueController 内部） |
| reset 语义 | `send(0, 0, false)` | `auto_aim_enable=false`（协议里与旧 reset 相反） |
| yaw 语义 | 角度直接下发 | 多圈连续角，内部自动映射到与 `imu_yaw_unwrapped` 最近的等效角 |
| 编译依赖 | 无 | Ceres + `~/TorqueController` 源码 |

## 4. 已知问题 1：yaw 目标 0.2s 延迟

TorqueController 的 MPC 对 yaw 目标做了**有意的延迟缓冲**：
参考序列使用延迟 `dt_control × N` 后的目标，当前默认
`dt_control=0.01s`、`mpc_pred_N=20`，即 **0.2s**。

影响：

- `pitch_target_angle`、`fire`、`auto_aim_enable` 都是直通、无延迟；
- 只有 `target_yaw` 的实际响应相对输入延后约 0.2s，切 torque 通道后预测/火控
  必须把这个时间差算进补偿，否则打点会偏。

当前 [configs/infantry.yaml](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/configs/infantry.yaml:88)
的 `extra_predict_time : 0.170`（预测器需要补偿的总时间）是按旧串口通道调的，
**切到 torque 后需要重新标定**。两条调整路径：

1. 保持 `mpc_pred_N=20`，把 `extra_predict_time` 加大约 0.2s（具体值实车标定）；
2. 改 `dt_control` 或 `mpc_pred_N` 缩短延迟（延迟 = `dt_control × N`），
   但会同时改变 MPC 的预测窗口，需要重新验证控制品质。

注意：`TorqueGimbalSender` 只是把 `pipeline` 算出的 `mcu_command_yaw` 透传给
`RobotController::set()`，它不会帮你补偿这 0.2s，补偿必须由预测器/配置完成。

## 5. 已知问题 2：`auto_aim_switch` 硬编码 true

[users/infantry.cpp](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/users/infantry.cpp:90)
主循环里自瞄开关仍是原逻辑硬编码：

```cpp
bool auto_aim_switch = true;  // 原代码硬编码 true；以后可从电控读取
```

该值只参与 `initial.auto_aim_switch` 传给预测器，两条通道目前都不真正读取电控开关。

现状：

- serial 通道：`io::Communication` 其实能解析到 MCU 数据，但 `io::State`
  没有把开关位带出来；
- torque 通道：`RobotController::State::McuData` 里**有** `auto_aim_switch`
  字段（电控发来的原始开关），但 `TorqueGimbalSender::state()` 目前没把它映射进
  `io::State`。

后续要做真实切换，建议把 `io::State` 增加一个
`auto_aim_switch`（或类似）字段，serial 通道从 MCU 帧解析、torque 通道从
`RobotController::getState()` 映射，主循环再删掉硬编码。

## 6. 本次改动涉及的文件

| 文件 | 改动 |
|---|---|
| [io/gimbal_io.hpp](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/io/gimbal_io.hpp) / [io/gimbal_io.cpp](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/io/gimbal_io.cpp) | 新增：`GimbalIo` 可切换下发模块（serial/torque 双通道） |
| [io/CMakeLists.txt](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/io/CMakeLists.txt) | 可选编入 TorqueController 源码（需 Ceres）；不编其 CRC.cpp |
| [io/communication/CRC.h](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/io/communication/CRC.h) / [io/communication/CRC.cpp](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/io/communication/CRC.cpp) | CRC8 改为 const 指针 + size_t；新增 CRC32（TorqueController IMU 协议用） |
| [configs/infantry.yaml](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/configs/infantry.yaml) | 新增 `command_channel` 与 `torque_controller:` 参数段 |
| [users/infantry.cpp](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/users/infantry.cpp) | `io::Communication` 换成 `io::GimbalIo`，统一走 `GimbalCommand` |
| [users/infantry_debug.cpp](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/users/infantry_debug.cpp) | 同上（调试版主程序） |
| [MODULES.md](/home/shang-beihang/transistor_rm2027_algorithm_visual_ws/MODULES.md) | io 层模块表补充 `gimbal_io` |

> 注：以上改动目前都还在工作区，未提交 git。
