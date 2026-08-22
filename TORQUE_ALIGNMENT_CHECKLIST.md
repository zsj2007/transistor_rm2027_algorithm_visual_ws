# TorqueController 与电控对齐清单

> 用途：和电控同学逐项确认 torque 通道（`0x42 0x52 0x02`）的协议与语义。
> 上位机代码依据：`io/communication/CRC.cpp`、`~/TorqueController/include/communication/Protocol.hpp`
> `~/TorqueController/include/communication/Communications.hpp`、`src/communication/McuDataPreprocessor.cpp`

## 0. 前置条件

- [ ] 电控固件是否已支持新力矩协议（前导 `0x42 0x52 0x02`），不是旧角度协议（`0x42 0x52 0xCD`）
- [ ] 两个串口：MCU 口（产品名 ≠ `AutoAim_IMU_Com`，如 `STM32 Virtual ComPort`）+ IMU 口（产品名 == `AutoAim_IMU_Com`）
- [ ] 波特率 115200，8 数据位，无校验，1 停止位

## 1. MCU 下发帧 TX（上位机 → 电控，28 字节）

| # | 字段 | 上位机实际内容 | 要电控确认的点 |
|---|---|---|---|
| 0~2 | 帧头 | `42 52 02` | 是否一致 |
| 3 | data_size | `23` | 是否一致 |
| 4 | auto_aim_enable | 0/1（**与旧 reset 相反**，1=使能） | 电控端语义 |
| 5 | fire | 0/1 | 是否直接驱动发弹/摩擦轮 |
| 6~9 | pitch_target_angle float | 上位机已线性映射：`20.523245 × pitch_rad + 0.475049` | 电控收到后的单位/范围（应约 ±π/2 对应 ±1.5V 或电机标定值？） |
| 10 | yaw_torque_only_mode | 0/1 | 0=力矩+位置+速度，1=仅力矩 |
| 11~18 | yaw_target_angle double | **MPC 预测角，rad，多圈不限位**（非视觉瞄准角） | 电控按绝对多圈角执行？ |
| 19~22 | yaw_target_velocity float | rad/s，MPC 预测速度 | 单位一致？ |
| 23~26 | yaw_torque float | -1.0 ~ 1.0 归一化力矩 | 电控侧限幅（参考 ±16384/16384）换算是否一致 |
| 27 | CRC8 | 初值 0xFF 查表，对前 27 字节 | 电控校验一致？ |

## 2. MCU 接收帧 RX（电控 → 上位机，37 字节）

- [ ] 电控是否**周期发送**该帧？帧率多少？（上位机融合/MPC 依赖它）

| # | 字段 | 上位机解析 | 要电控确认的点 |
|---|---|---|---|
| 0~3 | 帧头+长度 | `42 52 02 0x20`(32) | 一致 |
| 4~7 | bullet_velocity float | m/s 直接用 | 单位 m/s？ |
| 8~11 | pitch_angle float | 上位机按 `1.122635 × raw − 0.170755` 解为 rad | **电控发送的 raw 是什么单位？公式是否匹配**（与 TX 标定非互逆，重点） |
| 12~19 | yaw_angle double | **直接作为 rad 多圈角，零缩放** | **必须是 yaw 电机编码器多圈角，不要减 IMU 角，不要缩放** |
| 20~23 | yaw_omega float | rad/s | 单位 |
| 24~27 | chassis_imu_yaw float | rad | 底盘 IMU yaw（0~2π） |
| 28~31 | chassis_imu_omega float | rad/s | 底盘 IMU 角速度 |
| 32 | mark | 循环标志 | 是否递增 |
| 33 | color | 0=RED，其他=BLUE | 语义 |
| 34 | auto_aim_switch | 电控自瞄开关 | 电控是否真的发送 |
| 35 | yaw_temperature | yaw 电机温度 | 单位 ℃？ |
| 36 | CRC8 | 初值 0xFF 查表 | 一致 |

## 3. IMU 帧（前导 `A7 B6 C5`）

- [ ] IMU 口产品名 == `AutoAim_IMU_Com`，波特率 115200
- [ ] 下发 8B：心跳（前导 + data_size=0 + crc32）
- [ ] 接收 60B：`A7 B6 C5` + 长度 + gx/gy/gz/ax/ay/az（6×float）+ euler_yaw/pitch/roll（3×double）+ dt_one_tenth_ms（uint32，单位 0.1ms）+ crc32
- [ ] IMU 帧率是否 ~1kHz（融合需要高频）

## 4. 控制行为语义

- [ ] 上位机后台 **100Hz** 持续下发，电控端按帧更新执行即可
- [ ] yaw 目标内部有 **0.2s 延迟缓冲**（MPC 参考序列，`dt×N`），属于设计，不是故障
- [ ] 融合未就绪（IMU/电控数据缺失）时，上位机可能下发 **yaw=0、torque=0** 的帧；电控端建议忽略无效帧或由上位机用 auto_aim_enable=0 表示
- [ ] 无目标 reset 时：`auto_aim_enable=0`，角度保持最后值（新行为）或回 0（可配置 `no_target_hold_position`）
- [ ] 掉线/超时保护：上位机侧有离线判断，电控端是否也需要“超时无帧自动停车/回中”？

## 5. 必须落实的三件事

1. **pitch 收发标定**：下发 `20.523245×rad+0.475049`、回传 `1.122635×raw−0.170755` 两套系数与电控的发送/接收单位完全对齐（或电控按这套实现），否则 pitch 会偏；
2. **yaw_angle 语义**：电控回传必须是 yaw 电机编码器多圈角（rad），不加 IMU、不缩放；
3. **力矩换算**：`yaw_torque -1~1` 与电控实际驱动（PWM/电流）的映射关系确认一致。

## 6. 现场验证方法

- 用 `./build/torque_manual_test` 手动输入角度：`y 30` / `p -5`，看云台是否按预期转、方向是否正确；
- 串口抓包（如 `tcpdump` 不适用串口，用 MVS/串口助手或逻辑分析仪）逐字节对比 TX/RX 帧；
- 先 `s` 确认 `fused=1 mcu=1 imu=1`，再发角度命令。
