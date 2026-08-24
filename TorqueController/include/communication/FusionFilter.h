#ifndef FUSION_FILTER_H
#define FUSION_FILTER_H

#include <mutex>

// ============================================================================
// YawChassisFusion — 高频 yaw 轴与底盘姿态融合滤波器
//
// 输入（融合）:
//   - IMU（高频 ~1kHz，本地时钟积分）: euler_yaw/pitch/roll（world 系姿态，
//     无误差）、gx/gy/gz（绕 IMU 本体角速度）
//   - MCU（yaw 相关字段低频 <10Hz，pitch 近似实时）: yaw_angle（yaw 关节编码器，
//     多圈绝对）、yaw_omega（关节速度，MCU 直接测量，作低频速度基准）、
//     pitch_angle（pitch 关节角）
//
// 坐标系（遵循 RobotTfTree，ZXY 约定: R = Rz(yaw)·Rx(pitch)·Ry(roll)）:
//   链: chassis(yaw_c,pitch_c,roll_c) → yaw 关节 Rz(yaw_joint) → pitch 关节
//       Rx(pitch_joint) → head → imu（imu 与 head 固连，仅平移）
//   R_head = R_chassis · Rz(yaw_joint) · Rx(pitch_joint)
//   （物理 pitch 轴绕 x，由用户确认）
//
// 输出（高频，跟 IMU 包频率）:
//   - yaw_pos    : yaw 关节解卷绕位置（rad，多圈连续）
//   - yaw_rate   : yaw 关节速度（rad/s）
//   - chassis_*  : 底盘 world 系欧拉角（yaw 解卷绕，pitch/roll 为 ZXY 反解值）
//   - imu_yaw_unwrapped : IMU euler yaw 解卷绕（多圈连续）
//
// 算法（不使用任何差分计算速度）:
//   - 底盘姿态: R_chassis = R_head · (Rz(yaw_joint)·Rx(pitch_joint))ᵀ，
//     用 MCU yaw_angle（多圈绝对）反解（避免依赖 yaw_pos 造成反馈）
//   - yaw 关节速度（互补滤波，无差分）:
//     yaw_rate_proj = gy·sin(pitch_joint) + gz·cos(pitch_joint)
//     （yaw 关节轴在 IMU 本体系 = (0, sin p, cos p)）
//     yaw_rate = yaw_rate_proj − 低通(yaw_rate_proj − yaw_omega)
//     —— 高频来自 IMU gyro 投影，低频基准来自 MCU yaw_omega（直接测量），
//        低通项吸收 gyro 偏置与低频底盘贡献，不做任何数值差分
//   - yaw 位置: 高频积分 + MCU yaw_angle（多圈绝对）慢速位置级校正（消漂移）
//   - bias（imu_yaw_unw − yaw_pos − chassis_yaw）持续修正：静止时快速、
//     运动时缓慢，向反解隐含偏置收敛（补偿 yaw_pos 缓慢漂移）
//
// imu_weight（构造参数，默认 1.0）: 直接控制的 IMU 数据的信任权重。
//   权重越高 → 所有低频（MCU）校正强度越低（各校正增益统一 ÷ imu_weight）：
//     - 位置级校正 KP（MCU yaw_angle 绝对位置）
//     - 速度互补滤波低通系数（MCU yaw_omega 低频基准）
//     - bias 修正增益（MCU 反解驱动）
//   默认 1.0 即完全信任 IMU；<1 更信 MCU（校正更强），>1 更信 IMU（校正更弱）。
// ============================================================================

class YawChassisFusion {
public:
    // 严格反解数据包（独立输出，非 Output 子结构）：
    // 使用当前 IMU 欧拉角（始终为 imu 传来的数据，缺失时为 0）+ pitch_angle + yaw_pos
    // 严格反解底盘欧拉角，保证 R_imu = R_chassis·Rz(yaw_pos)·Rx(pitch_angle) 恒成立。
    // 包内所有角度均缠绕到 (-π, π]；数据始终有效，所需数据缺失时以 0 参与计算。
    struct StrictPose {
        // ── 反解输入快照（wrap 到 (-π, π]）──
        double imu_euler_yaw = 0.0;
        double imu_euler_pitch = 0.0;
        double imu_euler_roll = 0.0;
        double yaw_pos = 0.0;       // 反解所用的 yaw 关节位置（wrap 后）
        double pitch_angle = 0.0;   // 反解所用的 pitch 关节角（wrap 后）
        // ── 严格反解结果（wrap 到 (-π, π]）──
        double chassis_yaw = 0.0;
        double chassis_pitch = 0.0;
        double chassis_roll = 0.0;
    };

    struct Output {
        bool   valid = false;
        double yaw_pos = 0.0;            // yaw 关节解卷绕位置 (rad, 多圈)
        double yaw_rate = 0.0;           // yaw 关节速度 (rad/s)
        double chassis_yaw = 0.0;        // 底盘 world 系 yaw（解卷绕）
        double chassis_pitch = 0.0;      // 底盘 world 系 pitch (rad)
        double chassis_roll = 0.0;       // 底盘 world 系 roll (rad)
        double imu_yaw_unwrapped = 0.0;  // IMU euler yaw 解卷绕 (rad)
    };

    // imu_weight: 直接控制的 IMU 数据信任权重（>0，默认 1.0）
    explicit YawChassisFusion(double imu_weight = 5.0);

    // 高频路径：每个 IMU 包调用（内部用本地时钟积分）
    void onImu(double euler_yaw, double euler_pitch, double euler_roll,
               double gx, double gy, double gz);

    // 低频路径：每个 MCU 包调用（yaw 字段值变化检测做位置校正；
    // chassis_imu_yaw/omega 为底盘 IMU 数据，水平假设下直接利用：
    //   yaw_rate = gy·sin p + gz·cos p − chassis_imu_omega − gyro偏置
    //   bias 修正用 chassis_imu_yaw（底盘 yaw 直接观测））
    void onMcu(double yaw_angle, double yaw_omega, double pitch_angle,
               double chassis_imu_yaw, double chassis_imu_omega);

    // 读取融合输出（线程安全：回调线程写、主线程读）
    Output output() const;

    // 读取严格反解数据包（线程安全；读取时基于当前状态即时计算，
    // 始终有效，缺失数据以 0 参与，所有角度 wrap 到 (-π, π]）
    StrictPose strictPose() const;

    // 重新锚定 imu_yaw_unwrapped：将其设到与 yaw_pos 夹角最近的圈内
    // （|差| ≤ π）。用于解卷绕错位（imu_yaw_unwrapped 与 yaw_pos 相差数圈）后的修复；
    // 同时补偿 bias_，保持 chassis_yaw 输出连续。线程安全。
    void reanchorImuYaw();

    // 复位（重上电/重连时调用）
    void reset();

private:
    static double nowSeconds();

    // 解卷绕：将 angle（卷绕到 ~(-π,π]）延展为与 prev 连续的绝对值
    static double unwrapTo(double angle, double prev, double& corr);

    // 状态
    mutable std::mutex mtx_;    // 保护输出与内部状态
    Output out_;
    double imu_weight_ = 1.0;         // 直接控制的 IMU 数据信任权重
    double pos_corr_kp_eff_ = 1.0;    // 有效位置校正增益（÷ imu_weight）
    double diff_lp_alpha_eff_ = 0.02; // 有效互补滤波低通系数（÷ imu_weight）
    double bias_kp_still_eff_ = 0.5;  // 有效 bias 修正增益-静止（÷ imu_weight）
    double bias_kp_moving_eff_ = 0.05;// 有效 bias 修正增益-运动（÷ imu_weight）
    double yaw_pos_ = 0.0;            // yaw 关节位置（积分+校正，多圈）
    double yaw_rate_ = 0.0;
    double yaw_rate_proj_ = 0.0;      // gy·sin p + gz·cos p（投影，供静止判定/调试）
    double diff_lp_ = 0.0;            // 低通(yaw_rate_proj − yaw_omega − chassis_imu_omega)（gyro 偏置）
    double yaw_omega_last_ = 0.0;     // 最近 MCU yaw_omega（直接测量，低频基准）
    double chassis_imu_omega_ = 0.0;  // 最近 MCU chassis_imu_omega（底盘 yaw 角速度）
    double pitch_joint_ = 0.0;        // 最近 MCU pitch（实时）
    double chassis_pitch_ = 0.0;      // 反解底盘 pitch/roll（MCU 更新时刷新）
    double chassis_roll_ = 0.0;
    double bias_ = 0.0;               // imu_yaw_unw − yaw_pos − chassis_yaw（持续修正）
    bool   bias_init_ = false;
    double imu_yaw_unwrapped_ = 0.0;
    double imu_yaw_corr_ = 0.0;       // imu yaw 解卷绕累计修正
    double chassis_yaw_unwrapped_ = 0.0;  // 反解底盘 yaw（解卷绕，供输出/差分基准）
    double chassis_yaw_corr_ = 0.0;   // 反解底盘 yaw 解卷绕累计修正
    double chassis_imu_yaw_unw_ = 0.0;    // 底盘 IMU yaw（解卷绕，bias 修正观测）
    double chassis_imu_yaw_corr_ = 0.0;   // 底盘 IMU yaw 解卷绕累计修正

    double yaw_angle_last_ = 0.0;     // 最近 MCU yaw_angle（多圈绝对）
    bool   have_yaw_angle_ = false;
    bool   pos_init_from_mcu_ = false;    // 已用 MCU yaw_angle 初始化位置基准

    double last_imu_t_ = -1.0;        // 上次 IMU 包本地时间（积分用）
    double last_euler_yaw_ = 0.0;     // 最近 IMU 欧拉角缓存（onMcu 反解用）
    double last_euler_pitch_ = 0.0;
    double last_euler_roll_ = 0.0;

    // 位置级校正基值增益（1/s）：误差以 exp(-Kp·t) 衰减，Kp=1 → 时间常数 1s
    static constexpr double POS_CORR_KP = 1.0;
    // 互补滤波低通基值系数（@1kHz 输入）：alpha=0.02 → 时间常数 ~50ms（截止 ~3Hz），
    // 底盘 yaw 运动通常低频，被低通项吸收；高频瞬态保留 gyro 投影
    static constexpr double DIFF_LP_ALPHA = 0.02;
    // bias 修正基值增益：静止时快速（0.5/次 MCU 更新），运动时缓慢（0.05/次）
    static constexpr double BIAS_KP_STILL = 0.5;
    static constexpr double BIAS_KP_MOVING = 0.05;
    // 静止判定阈值 (rad/s)
    static constexpr double STILL_RATE = 0.05;
};

#endif // FUSION_FILTER_H
