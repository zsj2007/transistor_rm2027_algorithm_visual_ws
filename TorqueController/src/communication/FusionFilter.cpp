#include "communication/FusionFilter.h"

#include <cmath>
#include <chrono>
#include <algorithm>

namespace {

// ============================================================================
// 3x3 旋转矩阵工具（ZXY 约定: R = Rz(yaw)·Rx(pitch)·Ry(roll)，与
// RobotTfTree/CoordinateTransform 矩阵一致，pitch 绕 x）
// ============================================================================
struct Mat3 {
    double m[3][3];
};

Mat3 eulerZXY(double yaw, double pitch, double roll) {
    double cy = std::cos(yaw), sy = std::sin(yaw);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cr = std::cos(roll), sr = std::sin(roll);
    Mat3 R;
    R.m[0][0] = cy*cr - sy*sp*sr;  R.m[0][1] = -sy*cp;         R.m[0][2] = cy*sr + sy*sp*cr;
    R.m[1][0] = sy*cr + cy*sp*sr;  R.m[1][1] =  cy*cp;         R.m[1][2] = sy*sr - cy*sp*cr;
    R.m[2][0] = -cp*sr;            R.m[2][1] =  sp;            R.m[2][2] = cp*cr;
    return R;
}

Mat3 rotZ(double y) {
    double cy = std::cos(y), sy = std::sin(y);
    Mat3 R;
    R.m[0][0] = cy;  R.m[0][1] = -sy; R.m[0][2] = 0.0;
    R.m[1][0] = sy;  R.m[1][1] =  cy; R.m[1][2] = 0.0;
    R.m[2][0] = 0.0; R.m[2][1] = 0.0; R.m[2][2] = 1.0;
    return R;
}

// pitch 关节绕 x 轴（物理轴，用户确认）
Mat3 rotX(double p) {
    double cp = std::cos(p), sp = std::sin(p);
    Mat3 R;
    R.m[0][0] = 1.0; R.m[0][1] = 0.0; R.m[0][2] = 0.0;
    R.m[1][0] = 0.0; R.m[1][1] = cp;  R.m[1][2] = -sp;
    R.m[2][0] = 0.0; R.m[2][1] = sp;  R.m[2][2] = cp;
    return R;
}

Mat3 mul(const Mat3& A, const Mat3& B) {
    Mat3 R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            R.m[i][j] = A.m[i][0]*B.m[0][j] + A.m[i][1]*B.m[1][j] + A.m[i][2]*B.m[2][j];
        }
    return R;
}

Mat3 trans(const Mat3& A) {
    Mat3 R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R.m[i][j] = A.m[j][i];
    return R;
}

// ZXY 欧拉角提取（R = Rz·Rx·Ry）:
//   pitch = asin(M21), yaw = atan2(-M01, M11), roll = atan2(-M20, M22)
void matToEulerZXY(const Mat3& R, double& yaw, double& pitch, double& roll) {
    pitch = std::asin(std::max(-1.0, std::min(1.0, R.m[2][1])));
    const double eps = 1e-6;
    if (std::fabs(std::cos(pitch)) > eps) {
        yaw = std::atan2(-R.m[0][1], R.m[1][1]);
        roll = std::atan2(-R.m[2][0], R.m[2][2]);
    } else {
        roll = 0.0;
        yaw = std::atan2(R.m[1][0], R.m[0][0]);
    }
}

} // namespace

double YawChassisFusion::nowSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// imu_weight 越高 → 所有低频（MCU）校正强度越低（各增益 ÷ imu_weight）
YawChassisFusion::YawChassisFusion(double imu_weight) {
    imu_weight_ = (imu_weight > 0.0) ? imu_weight : 1.0;
    pos_corr_kp_eff_     = std::clamp(POS_CORR_KP   / imu_weight_, 0.01, 100.0);
    diff_lp_alpha_eff_   = std::clamp(DIFF_LP_ALPHA / imu_weight_, 0.0001, 1.0);
    bias_kp_still_eff_   = std::clamp(BIAS_KP_STILL / imu_weight_, 0.01, 10.0);
    bias_kp_moving_eff_  = std::clamp(BIAS_KP_MOVING/ imu_weight_, 0.001, 10.0);
}

double YawChassisFusion::unwrapTo(double angle, double prev, double& corr) {
    double val = angle + corr;
    double diff = val - prev;
    while (diff > M_PI)  { corr -= 2.0 * M_PI; diff -= 2.0 * M_PI; }
    while (diff < -M_PI) { corr += 2.0 * M_PI; diff += 2.0 * M_PI; }
    return angle + corr;
}

void YawChassisFusion::reset() {
    std::lock_guard<std::mutex> lock(mtx_);
    out_ = Output{};
    yaw_pos_ = 0.0;
    yaw_rate_ = 0.0;
    yaw_rate_proj_ = 0.0;
    diff_lp_ = 0.0;
    yaw_omega_last_ = 0.0;
    pitch_joint_ = 0.0;
    chassis_pitch_ = 0.0;
    chassis_roll_ = 0.0;
    bias_ = 0.0;
    bias_init_ = false;
    imu_yaw_unwrapped_ = 0.0;
    imu_yaw_corr_ = 0.0;
    chassis_yaw_unwrapped_ = 0.0;
    chassis_yaw_corr_ = 0.0;
    chassis_imu_yaw_unw_ = 0.0;
    chassis_imu_yaw_corr_ = 0.0;
    yaw_angle_last_ = 0.0;
    have_yaw_angle_ = false;
    pos_init_from_mcu_ = false;
    chassis_imu_omega_ = 0.0;
    last_imu_t_ = -1.0;
    last_euler_yaw_ = 0.0;
    last_euler_pitch_ = 0.0;
    last_euler_roll_ = 0.0;
}

YawChassisFusion::Output YawChassisFusion::output() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return out_;
}

// 严格反解数据包：读取时即时计算。所有角度 wrap 到 (-π, π]；
// 所需数据缺失时内部状态为 0，以 0 参与计算（始终有效）；
// imu 欧拉角恒为 imu 传来的数据（last_euler_* 由 onImu 缓存）。
YawChassisFusion::StrictPose YawChassisFusion::strictPose() const {
    std::lock_guard<std::mutex> lock(mtx_);
    StrictPose sp;

    // ── 反解输入（wrap 到 (-π, π]）──
    double yi = std::remainder(last_euler_yaw_, 2.0 * M_PI);
    double pi = std::remainder(last_euler_pitch_, 2.0 * M_PI);
    double ri = std::remainder(last_euler_roll_, 2.0 * M_PI);
    // yaw_pos 仅在 MCU yaw_angle 基准到位后有效（有绝对多圈基准）；
    // MCU 数据缺失时视为缺失，用 0 参与计算（此时底盘欧拉角 = imu 欧拉角）
    double yp = have_yaw_angle_ ? std::remainder(yaw_pos_, 2.0 * M_PI) : 0.0;
    double pa = std::remainder(pitch_joint_, 2.0 * M_PI);
    sp.imu_euler_yaw = yi;
    sp.imu_euler_pitch = pi;
    sp.imu_euler_roll = ri;
    sp.yaw_pos = yp;
    sp.pitch_angle = pa;

    // ── 严格反解（wrap 不影响 sin/cos，结果等价）──
    Mat3 R_head = eulerZXY(yi, pi, ri);
    Mat3 R_chain = mul(rotZ(yp), rotX(pa));
    Mat3 R_ch = mul(R_head, trans(R_chain));
    double cy, cp_, cr;
    matToEulerZXY(R_ch, cy, cp_, cr);
    sp.chassis_yaw   = std::remainder(cy, 2.0 * M_PI);
    sp.chassis_pitch = std::remainder(cp_, 2.0 * M_PI);
    sp.chassis_roll  = std::remainder(cr, 2.0 * M_PI);
    return sp;
}

// 重新锚定 imu_yaw_unwrapped 到与 yaw_pos 夹角最近的圈内（|差| ≤ π）。
// 同时把 bias_ 调整相同的整圈量，保持 chassis_yaw 输出连续。
void YawChassisFusion::reanchorImuYaw() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!have_yaw_angle_) return;   // 尚无 yaw_pos 绝对基准

    double new_val = yaw_pos_ + std::remainder(imu_yaw_unwrapped_ - yaw_pos_, 2.0 * M_PI);
    double delta = new_val - imu_yaw_unwrapped_;   // 整圈数（2π 的整数倍）
    if (delta == 0.0) return;

    imu_yaw_unwrapped_ = new_val;
    imu_yaw_corr_ = imu_yaw_unwrapped_ - last_euler_yaw_;   // 保持后续解卷绕一致
    bias_ += delta;   // chassis_yaw = imu_yaw_unw − yaw_pos − bias 保持不变
}

// ============================================================================
// 高频路径（每个 IMU 包，~1kHz，本地时钟积分）
// ============================================================================
void YawChassisFusion::onImu(double euler_yaw, double euler_pitch, double euler_roll,
                             double gx, double gy, double gz) {
    (void)gx;  // yaw 轴投影不依赖 gx（pitch 绕 x，yaw 轴在 IMU 系 = (0, sin p, cos p)）

    std::lock_guard<std::mutex> lock(mtx_);

    // 缓存最近 IMU 欧拉角（onMcu 反解用，时间错位 <1 帧）
    last_euler_yaw_ = euler_yaw;
    last_euler_pitch_ = euler_pitch;
    last_euler_roll_ = euler_roll;

    double now = nowSeconds();
    if (last_imu_t_ < 0.0) {
        last_imu_t_ = now;   // 首帧：建立时间基准
        return;
    }
    double dt = now - last_imu_t_;
    last_imu_t_ = now;
    if (dt <= 0.0 || dt > 0.1) {
        return;   // 异常间隔（丢包/暂停）
    }

    // ---- 1. IMU euler yaw 解卷绕（多圈连续）----
    imu_yaw_unwrapped_ = unwrapTo(euler_yaw, imu_yaw_unwrapped_, imu_yaw_corr_);

    // ---- 2. yaw 关节速度（互补滤波，无差分；显式利用底盘 IMU 角速度）----
    // yaw 关节轴在 IMU 本体系 = (0, sin p, cos p)（pitch 绕 x）；
    // 投影 = yaw_rate + chassis_imu_omega + gyro 偏置（水平假设下物理和）。
    // 显式减去底盘 IMU 角速度（低频），低通差分项仅吸收 gyro 偏置。
    double sp = std::sin(pitch_joint_), cp = std::cos(pitch_joint_);
    yaw_rate_proj_ = gy * sp + gz * cp;
    double inst_diff = yaw_rate_proj_ - yaw_omega_last_ - chassis_imu_omega_;
    diff_lp_ += diff_lp_alpha_eff_ * (inst_diff - diff_lp_);
    yaw_rate_ = yaw_rate_proj_ - chassis_imu_omega_ - diff_lp_;

    // ---- 3. 位置积分（多圈）+ 慢速位置校正 ----
    yaw_pos_ += yaw_rate_ * dt;
    if (have_yaw_angle_) {
        if (!pos_init_from_mcu_) {
            yaw_pos_ = yaw_angle_last_;   // 首次拿绝对基准直接对齐
            pos_init_from_mcu_ = true;
        } else {
            yaw_pos_ += pos_corr_kp_eff_ * (yaw_angle_last_ - yaw_pos_) * dt;
        }
    }

    // ---- 4. 底盘姿态（高频输出）----
    // yaw：开环外推 imu_yaw_unw − yaw_pos − bias（bias 持续修正，
    //       无差分环节，yaw_pos 误差由位置校正限制，不会正反馈发散）
    // pitch/roll：MCU 更新时反解刷新（底盘俯仰/横滚变化慢）
    double chassis_yaw_hi = (bias_init_ && have_yaw_angle_)
                                ? (imu_yaw_unwrapped_ - yaw_pos_ - bias_) : 0.0;

    out_.valid = have_yaw_angle_;
    out_.yaw_pos = yaw_pos_;
    out_.yaw_rate = yaw_rate_;
    out_.chassis_yaw = chassis_yaw_hi;
    out_.chassis_pitch = chassis_pitch_;
    out_.chassis_roll = chassis_roll_;
    out_.imu_yaw_unwrapped = imu_yaw_unwrapped_;
}

// ============================================================================
// 低频路径（每个 MCU 包；yaw 字段低频更新 <10Hz，pitch 实时）
// ============================================================================
void YawChassisFusion::onMcu(double yaw_angle, double yaw_omega, double pitch_angle,
                             double chassis_imu_yaw, double chassis_imu_omega) {
    std::lock_guard<std::mutex> lock(mtx_);

    pitch_joint_ = pitch_angle;   // pitch 近似实时
    chassis_imu_omega_ = chassis_imu_omega;   // 底盘 yaw 角速度（低频，随包更新）

    // yaw 字段更新检测（重复包值完全相同）
    if (have_yaw_angle_ && std::fabs(yaw_angle - yaw_angle_last_) <= 1e-9) {
        return;   // 非新数据
    }

    // 首次收到 MCU yaw_angle：
    //   - imu_yaw_unwrapped 初始化到"与 yaw_angle 同一圈、角度为最近 IMU 的 yaw"
    //     （此后解卷绕基准与 MCU 多圈基准一致，而非从 0 开始累积）
    //   - yaw_pos 直接对齐到 yaw_angle（避免首次 bias 标定受未对齐积分值污染）
    if (!have_yaw_angle_) {
        double diff = std::remainder(last_euler_yaw_ - yaw_angle, 2.0 * M_PI);
        imu_yaw_unwrapped_ = yaw_angle + diff;
        imu_yaw_corr_ = imu_yaw_unwrapped_ - last_euler_yaw_;
        yaw_pos_ = yaw_angle;
        pos_init_from_mcu_ = true;
    }

    yaw_angle_last_ = yaw_angle;
    yaw_omega_last_ = yaw_omega;   // 关节速度基准（MCU 直接测量，非差分）
    have_yaw_angle_ = true;

    // ---- 反解底盘姿态（用 MCU yaw_angle 绝对基准，避免依赖 yaw_pos 造成反馈）----
    // R_chassis = R_head(最近 IMU 欧拉角) · (Rz(yaw_angle)·Rx(pitch))ᵀ
    Mat3 R_head = eulerZXY(last_euler_yaw_, last_euler_pitch_, last_euler_roll_);
    Mat3 R_chain = mul(rotZ(yaw_angle), rotX(pitch_angle));
    Mat3 R_chassis = mul(R_head, trans(R_chain));

    double yaw_c, pitch_c, roll_c;
    matToEulerZXY(R_chassis, yaw_c, pitch_c, roll_c);
    double chassis_yaw_unw = unwrapTo(yaw_c, chassis_yaw_unwrapped_, chassis_yaw_corr_);
    chassis_yaw_unwrapped_ = chassis_yaw_unw;
    chassis_pitch_ = pitch_c;
    chassis_roll_ = roll_c;

    // ---- 底盘 IMU yaw 解卷绕（bias 修正的直接观测）----
    // chassis_imu_yaw 为 0~2π 卷绕值，连续化后代表底盘 yaw
    chassis_imu_yaw_unw_ = unwrapTo(chassis_imu_yaw, chassis_imu_yaw_unw_, chassis_imu_yaw_corr_);

    // ---- bias 持续修正 ----
    // bias_target = imu_yaw_unw − yaw_pos − chassis_imu_yaw_unw
    // （底盘 IMU 直接测量底盘 yaw，水平假设下无漂移；向它慢速收敛以
    //   补偿 yaw_pos 的缓慢漂移；静止时快速、运动时缓慢）
    // （仅当 yaw_pos 已有基准时修正才有意义；首次直接全量标定）
    if (have_yaw_angle_) {
        double bias_target = imu_yaw_unwrapped_ - yaw_pos_ - chassis_imu_yaw_unw_;
        bool still = (std::fabs(yaw_rate_proj_) < STILL_RATE) &&
                     (std::fabs(yaw_rate_) < STILL_RATE);
        double kb = !bias_init_ ? 1.0 : (still ? bias_kp_still_eff_ : bias_kp_moving_eff_);
        bias_ += kb * (bias_target - bias_);
        bias_init_ = true;
    }
}
