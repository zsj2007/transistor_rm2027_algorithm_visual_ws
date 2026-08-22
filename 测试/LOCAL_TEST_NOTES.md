# 本机（实验室）相机联调记录 — 2026-08-21

> 目的：记录为了在这台机器上用海康 GigE 相机把自瞄跑通所做的临时改动，
> 明天上车测试前对照检查，避免忘记。

## ⚠️ 最关键的临时改动

**`configs/infantry.yaml` → `MIN_TRACKING_COUNT : 1`（原值 2）**

- 原因：本机相机链路只有 ~6fps 且检出断续，追踪器要求连续 2 帧匹配，
  计数永远凑不满 → 目标确认不了 → 解算/下发全 0。
- 上车前：**确认相机帧率正常（30fps+）后，把这一项改回 2**；
  否则单帧误检可能直接触发瞄准/开火。

## 其他配置改动（当前值 → 说明）

| 配置项 | 当前值 | 说明 |
|---|---|---|
| `camera_ExposureTime` | 100000.0 | 临时拉高便于暗环境看画面；正常应 5000~20000µs |
| `camera_Gain` | 23.0 | 相机增益范围 0~23.98；设 24 会超范围导致连不上相机 |
| `log_send_commands` | true | 每帧把实际下发指令打到控制台（`[SEND]`），排障用，上车可关 |
| `MIN_TRACKING_COUNT` | 1 | 临时，见上 |
| `no_target_hold_position` | false | 无目标时云台行为：false=回 0 位（原行为），true=保持最后位置不动 |

## 代码改动（已编译进 build/）

- `io/communication/Com.cpp` / `Com.h`：`sendData` 重写；`log_send_commands`
  打开时即使无串口也把完整 12 字节 TX 帧写入 debug 日志；无串口不再静默无日志。
- `io/gimbal_io.cpp` / `.hpp`：新增 `log_send_commands` 配置、每帧 `[SEND]`
  info 日志、`[SEND-JUMP]` 命令突变告警（>10°）。
- `io/camera/cameraConnect.cpp`：曝光/增益/自动模式设置失败改为警告继续，
  不再因单个参数超范围阻断相机连接。
- `io/gimbal_io.cpp` `TorqueGimbalSender::state()`：全部改走 `strict`
  严格反解包（无 valid 分支），`to_mcu_delta_*` 保持 0。
- `users/infantry.cpp` / `infantry_debug.cpp`：`torque_controller.yaw_torque_only_mode`
  与 `integral_enable` 从配置读入 cmd 并随帧下发（默认 false，serial 通道忽略）；
  弹速/敌我颜色在 MCU 数据无效时恢复配置默认值兜底。

## 硬件 / 网络要点（本机）

- 相机：海康 GigE，相机 `192.168.10.10`，PC `192.168.10.25`
  （USB 转网口适配器 `enxec1ac302c492`）。
- **当前网线协商成 100Mbps 半双工 → 相机只有 ~6fps**
  （1440x1080 Bayer8 ≈1.55MB/帧，顶满百兆带宽）。上车前检查千兆协商
  （`/sys/class/net/enxec1ac302c492/speed` 应为 1000、`duplex` 应为 full），
  否则帧率上不去，追踪器/预测都会受影响。
- IP `192.168.10.25` 在链路断开后会丢，需要重新添加：
  `sudo ip addr add 192.168.10.25/24 dev enxec1ac302c492`。
- MVS 会独占相机：跑算法前必须完全退出 MVS（本机 MVS 不稳定会崩溃，需 `kill -9`）。
- 相机输出 1440x1080，而 `camera_matrix` 仍是 1280x1024 的标定值——
  **内参不匹配尚未解决**，上车前要对齐（分辨率或内参），否则解算角度会偏。
