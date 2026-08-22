# 串口下发协议整理（serial 通道）

> 给电控同学对齐用。仓库支持两套下发通道：**旧下发**（`command_channel: serial`，`0x42 0x52 0xCD` 角度协议）与**新下发**（`command_channel: torque`，TorqueController MPC 力矩协议，MCU 帧前导 `0x42 0x52 0x02`），均为 115200 8N1。本文档同时整理两套。

## 1. 下发链路

```
流水线 Stage3（预测/火控）
  → mcu_command_pitch / mcu_command_yaw（弧度）
  → should_send_reset、fire_flag
    ↓
主循环 users/infantry.cpp
  → io::GimbalCommand{ pitch(rad), yaw(rad), fire }
    ↓
io::GimbalIo（serial 通道）
  → SerialGimbalSender::send
    ↓
io::Communication::send(pitch, yaw, fire)
    ↓
SerialCommunicationClass::sendData
    ↓
串口 TX 帧（12 字节）
```

说明：

- `command_channel` 支持 `serial`（本协议）和 `torque`（TorqueController MPC 力矩协议），当前用 `serial`。
- serial 通道**只发 pitch / yaw / fire**，旧协议没有“使能/自瞄开关”字段；`should_send_reset` 时主循环不填角度，按 `(0, 0, fire=false)` 下发。
- 串口选择：`configs/infantry.yaml` 的 `serial_port` 可显式指定（如 `/dev/ttyACM1`），留空则自动检测（跳过产品名含 IMU 的串口）。

## 2. TX 帧（上位机 → 电控，固定 12 字节）

| 偏移 | 长度 | 字段 | 内容 |
|---|---|---|---|
| 0 | 1 | 帧头 | `0x42` |
| 1 | 1 | 帧头 | `0x52` |
| 2 | 1 | 命令码 | `0xCD` |
| 3 | 1 | 数据长度 | `0x07`（= 4+2+1） |
| 4~7 | 4 | pitch | `float` 小端，值 = 角度(°) × 0.01/30（即 **30° 对应 0.01**，代码注释口径，待电控确认） |
| 8~9 | 2 | yaw | `int16` 小端，值 = 弧度 × 4096/π，环绕到 [-4096, 4095]（**4096 = π rad，一圈 8192**） |
| 10 | 1 | fire | `0x00` / `0x01` |
| 11 | 1 | CRC8 | 对第 0~10 字节（共 11 字节）查表计算 |

## 3. RX 帧（电控 → 上位机，长度字节 + 5 字节）

帧头同样为 `0x42 0x52 0xCD`，第 3 字节是数据长度（当前解析为 17），帧尾 1 字节 CRC8。

| 偏移 | 长度 | 字段 | 含义 / 换算 |
|---|---|---|---|
| 0~2 | 3 | 帧头 | `0x42 0x52 0xCD` |
| 3 | 1 | 数据长度 | `0x11`（17） |
| 4~7 | 4 | bullet_velocity | `float` 弹速 |
| 8~11 | 4 | bullet_angle | `float` 俯仰角：值 × 30/1.8 = 角度(°)，即 **1.8 对应 30°** |
| 12~13 | 2 | gimbal_yaw | `int16`：值 × π/4096 = 弧度，**4096 = π rad** |
| 14~15 | 2 | mark | `uint16`（当前代码只解析未使用） |
| 16 | 1 | color | 敌我颜色：0 = RED，其他 = BLUE |
| 17~20 | 4 | z_rotation_velocity | `float` 自转速度 |
| 21 | 1 | CRC8 | 对第 0~20 字节（共 21 字节）查表计算 |

总帧长 = `data_length + 5`；`data_length` 上限 64。

## 4. CRC8

- 查表法，**初值 `0xFF`**，逐字节 `index = crc ^ byte` 查表更新；
- 表格与函数在 `io/communication/CRC.cpp` / `CRC.h`，电控端如需对齐以该表为准。

## 5. 换算速查

| 方向 | 物理量 | 协议值 |
|---|---|---|
| 下发 | pitch 30° | 0.01（float） |
| 下发 | yaw π rad（180°） | 4096（int16） |
| 下发 | yaw 一圈（360°） | 8192（int16） |
| 上行 | pitch 30° | 1.8（float） |
| 上行 | yaw π rad | 4096（int16） |

波特率 115200，8 数据位，无校验，1 停止位。

## 6. 当前需要和电控对齐的点

1. **串口没数据**：上位机日志 `No data received, trying reconnect` 表示在选中的串口上收不到符合 `0x42 0x52 0xCD` 的合法帧。本机有两个口：
   - `/dev/ttyACM0` = `AutoAim_IMU_Com`（IMU，不应作为主串口）
   - `/dev/ttyACM1` = `STM32 Virtual ComPort`（电控主串口，配置 `serial_port: "/dev/ttyACM1"` 使用）
2. 请电控确认上行帧是否按第 3 节结构**周期发送**（帧头 + 长度 17 + 17 字节 + CRC8）。
3. 请电控确认 TX 帧中 **pitch 的缩放约定**（代码按“30° 对应 0.01”发送）和 **yaw 定点约定**（4096 = π rad）是否与电控端一致。
4. 请电控确认 **CRC8 初值 0xFF 查表**是否与电控端一致。
5. 帧头/命令码上下行共用 `0x42 0x52 0xCD`，注意别和 IMU 口的数据混。

## 7. 相关文件

- `io/communication/Com.cpp` / `Com.h`：帧收发、解析
- `io/communication.cpp`：延迟对齐、状态计算、发送封装
- `io/gimbal_io.cpp`：通道选择（serial / torque）
- `users/infantry.cpp`：主循环下发
- `configs/infantry.yaml`：`command_channel`、`serial_port`、`serial_delay_time`

---

# 新下发通道（command_channel: torque，TorqueController MPC 力矩协议）

## 新通道代码在独立仓库 `~/TorqueController`（不在自瞄算法仓库内），算法仓库只负责通过 `TorqueGimbalSender` 调用。核心是 `RobotController`：上位机**同时占用两条串口**——电控（MCU）口 + IMU 口，IMU 高频融合、MPC 求解后由后台 **100Hz 线程**持续下发。

## N1. 下发链路

```
主循环 users/infantry.cpp
  → io::GimbalCommand{ auto_aim_enable, yaw_torque_only_mode, yaw(rad 多圈), pitch(rad), fire, integral_enable }
    ↓
io::GimbalIo（torque 通道）
  → TorqueGimbalSender::send
    ↓
RobotController::set(...)            // 只更新“最新目标”，不直接发
    ↓
McuMpcController 后台 100Hz 线程
  → 读最新目标 → MPC 求解 → 组 mcu::SendPacket → 串口发送
```

关键语义：

- `set()` 每次只更新目标，真正发送由后台 100Hz 线程（10ms 周期）完成。
- **yaw 目标有 0.2s 延迟**：目标进入内部延迟缓冲（`dt_control × N` = 0.01 × 20 = 0.2s），MPC 参考序列跟踪延迟后的目标（有意设计）。
- pitch / fire / auto_aim_enable / yaw_torque_only_mode / integral_enable 直接透传，无延迟。
- yaw 目标自动做“最近等效角”映射（相对 IMU 解卷绕角，多圈语义）。

## N2. 串口选择

`RobotCommunication` 同时开两条串口（115200 8N1），按产品名筛选：

| 通道 | 筛选条件 | 本机对应 |
|---|---|---|
| MCU（电控） | 产品名 ≠ `AutoAim_IMU_Com` | `/dev/ttyACM1`（STM32 Virtual ComPort） |
| IMU | 产品名 == `AutoAim_IMU_Com` | `/dev/ttyACM0` |

## N3. MCU 协议（前导 `0x42 0x52 0x02`，CRC8）

### N3.1 下发帧 TX（上位机 → 电控，`mcu::SendPacket`，28 字节）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0~2 | 3 | 前导 | `0x42 0x52 0x02` |
| 3 | 1 | data_size | 23 |
| 4 | 1 | auto_aim_enable | 0/1（与旧协议 reset 相反） |
| 5 | 1 | fire | 0/1 |
| 6~9 | 4 | pitch_target_angle | `float`，单位 rad，范围 ±π/2；发送前线性映射：`值 = send_pitch_scale × 角度 + send_pitch_offset`（默认 20.523245 / 0.475049） |
| 10 | 1 | yaw_torque_only_mode | 0 = 力矩+位置+速度；1 = 仅力矩 |
| 11~18 | 8 | yaw_target_angle | `double`，rad，多圈不限位（MPC 参考） |
| 19~22 | 4 | yaw_target_velocity | `float`，rad/s |
| 23~26 | 4 | yaw_torque | `float`，-1.0~1.0（归一化力矩） |
| 27 | 1 | crc8 | 对前 27 字节 |

### N3.2 接收帧 RX（电控 → 上位机，`mcu::ReceivePacket`，37 字节）

| 偏移 | 长度 | 字段 | 说明 |
|---|---|---|---|
| 0~2 | 3 | 前导 | `0x42 0x52 0x02` |
| 3 | 1 | data_size | 32 |
| 4~7 | 4 | bullet_velocity | `float`，m/s |
| 8~11 | 4 | pitch_angle | `float`，rad；接收后线性映射：`imu_pitch = recv_pitch_scale × 值 + recv_pitch_offset`（默认 1.122635 / -0.170755） |
| 12~19 | 8 | yaw_angle | `double`，rad，**yaw 电机编码器直接读出的多圈角**（不要减 IMU 角） |
| 20~23 | 4 | yaw_omega | `float`，rad/s |
| 24~27 | 4 | chassis_imu_yaw | `float`，0~2π |
| 28~31 | 4 | chassis_imu_omega | `float` |
| 32 | 1 | mark | 递增循环标志 |
| 33 | 1 | color | 颜色标志 |
| 34 | 1 | auto_aim_switch | 电控自瞄开关 |
| 35 | 1 | yaw_temperature | yaw 电机温度 |
| 36 | 1 | crc8 | 对前 36 字节 |

## N4. IMU 协议（前导 `0xA7 0xB6 0xC5`，CRC32）

### N4.1 下发帧 TX（心跳，8 字节）

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0~2 | 3 | `0xA7 0xB6 0xC5` |
| 3 | 1 | data_size = 0 |
| 4~7 | 4 | crc32 |

### N4.2 接收帧 RX（60 字节）

| 偏移 | 长度 | 字段 |
|---|---|---|
| 0~2 | 3 | `0xA7 0xB6 0xC5` |
| 3 | 1 | data_size |
| 4~27 | 24 | gx/gy/gz/ax/ay/az（各 4 字节 `float`） |
| 28~51 | 24 | euler_yaw / euler_pitch / euler_roll（各 8 字节 `double`） |
| 52~55 | 4 | dt_one_tenth_ms（`uint32`） |
| 56~59 | 4 | crc32 |

## N5. CRC

- CRC8：与旧协议相同，查表法、初值 `0xFF`。
- CRC32：STM32 HAL 兼容，多项式 `0x04C11DB7`，初值 `0xFFFFFFFF`。

## N6. 电控端要特别确认的点

1. 上行 MCU 帧是否按 N3.2 结构周期发送（`0x42 0x52 0x02` + data_size 32 + 32 字节载荷 + CRC8）。
2. `yaw_angle` 必须是 **yaw 电机编码器多圈角（rad）**，不要用 IMU 角或减掉 IMU 角度。
3. pitch 的收发各有一组线性标定参数（`send_pitch_scale/offset`、`recv_pitch_scale/offset`），默认值见 N3，标定工具在 `~/TorqueController/src/pitch_calibration.cpp`。
4. 电控端参考实现：`~/TorqueController/mcu_code_demo/yaw_control.c`（含 yaw 多圈解算、`yaw_torque_only_mode` 两种模式、力矩限幅 ±16384/16384）。
5. 上位机与 IMU 之间是心跳 + 高频姿态（N4），IMU 口产品名为 `AutoAim_IMU_Com`。

## N7. 相关代码位置

- `~/TorqueController/include/communication/Protocol.hpp`：字节级结构定义（最权威）
- `~/TorqueController/include/communication/Communications.hpp`：端口筛选、CRC 选择
- `~/TorqueController/src/mpc/mcu_mpc_controller.cpp`：100Hz 发送线程
- `~/TorqueController/mcu_code_demo/yaw_control.c`：电控端参考
- 自瞄仓库 `io/gimbal_io.cpp`：`TorqueGimbalSender`（通道封装）
- `configs/infantry.yaml` 的 `torque_controller:` 段：MPC 参数、标定参数
