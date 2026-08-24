#include "RobotController.h"

RobotController::RobotController(double dt_control, int N,
                                 double J, double tau_c, double b, double tau_d,
                                 double max_torque, double max_torque_rate,
                                 double Q, double R, double Rd, int max_iter,
                                 double integral_gain,
                                 const McuDataPreprocessor::LinearParams& mcu_linear_params,
                                 bool sequence_mode)
    : sequence_mode_(sequence_mode),
      comm_(mcu_linear_params),
      mcu_mpc_(&comm_, dt_control, N,
               J, tau_c, b, tau_d,
               max_torque, max_torque_rate,
               Q, R, Rd, max_iter, integral_gain)
{
    mcu_mpc_.start();   // 启动后台 100Hz 发送线程
}

RobotController::~RobotController() {
    // mcu_mpc_ 析构自动 stop + join 后台线程；comm_ 析构停止串口线程
}

RobotController::State RobotController::getState() {
    State st;

    // ── MCU / IMU 原始数据 ──
    auto raw = comm_.getLatestData();
    if (raw.mcu_valid) {
        const auto& m = raw.mcu_packet;
        st.mcu.valid            = true;
        st.mcu.bullet_velocity  = m.bullet_velocity;
        st.mcu.pitch_angle      = m.pitch_angle;
        st.mcu.yaw_angle        = m.yaw_angle;
        st.mcu.yaw_omega        = m.yaw_omega;
        st.mcu.chassis_imu_yaw  = m.chassis_imu_yaw;
        st.mcu.chassis_imu_omega = m.chassis_imu_omega;
        st.mcu.mark             = m.mark;
        st.mcu.color            = m.color;
        st.mcu.auto_aim_switch  = m.auto_aim_switch;
        st.mcu.yaw_temperature  = m.yaw_temperature;
    }
    if (raw.imu_valid) {
        const auto& im = raw.imu_packet;
        st.imu.valid            = true;
        st.imu.gx = im.gx; st.imu.gy = im.gy; st.imu.gz = im.gz;
        st.imu.ax = im.ax; st.imu.ay = im.ay; st.imu.az = im.az;
        st.imu.euler_yaw   = im.euler_yaw;
        st.imu.euler_pitch = im.euler_pitch;
        st.imu.euler_roll  = im.euler_roll;
        st.imu.dt_one_tenth_ms = im.dt_one_tenth_ms;
    }

    // ── FusionFilter 输出 ──
    auto f = comm_.getFused();
    st.fused.valid            = f.valid;
    st.fused.yaw_pos          = f.yaw_pos;
    st.fused.yaw_rate         = f.yaw_rate;
    st.fused.chassis_yaw      = f.chassis_yaw;
    st.fused.chassis_pitch    = f.chassis_pitch;
    st.fused.chassis_roll     = f.chassis_roll;
    st.fused.imu_yaw_unwrapped = f.imu_yaw_unwrapped;

    // 严格反解数据包（独立输出）
    auto sp = comm_.getStrictPose();
    st.strict.imu_euler_yaw    = sp.imu_euler_yaw;
    st.strict.imu_euler_pitch  = sp.imu_euler_pitch;
    st.strict.imu_euler_roll   = sp.imu_euler_roll;
    st.strict.yaw_pos          = sp.yaw_pos;
    st.strict.pitch_angle      = sp.pitch_angle;
    st.strict.chassis_yaw      = sp.chassis_yaw;
    st.strict.chassis_pitch    = sp.chassis_pitch;
    st.strict.chassis_roll     = sp.chassis_roll;

    // ── MPC 状态（含参考/预测序列与积分值）──
    auto m = mcu_mpc_.state();
    st.mpc.yaw_target_angle    = m.yaw_target_angle;
    st.mpc.yaw_target_velocity = m.yaw_target_velocity;
    st.mpc.yaw_torque          = m.yaw_torque;
    st.mpc.delayed_target      = m.delayed_target;
    st.mpc.integral            = m.integral;
    st.mpc.ref_sequence        = m.ref_sequence;
    st.mpc.pred_sequence       = m.pred_sequence;

    return st;
}

void RobotController::set(bool auto_aim_enable, bool yaw_torque_only_mode,
                          double target_yaw, double pitch_target_angle, bool fire,
                          bool integral_enable) {
    if (sequence_mode_) {
        throw std::runtime_error("RobotController: SEQUENCE mode selected, "
                                 "use sequence set() instead of single set()");
    }
    mcu_mpc_.set(auto_aim_enable, yaw_torque_only_mode, target_yaw,
                 pitch_target_angle, fire, integral_enable);
}

void RobotController::set(bool auto_aim_enable, bool yaw_torque_only_mode,
                          const std::vector<double>& target_yaw_seq,
                          const std::vector<double>& pitch_seq,
                          const std::vector<bool>& fire_seq,
                          bool integral_enable) {
    if (!sequence_mode_) {
        throw std::runtime_error("RobotController: SINGLE mode selected, "
                                 "use single set() instead of sequence set()");
    }
    mcu_mpc_.set(auto_aim_enable, yaw_torque_only_mode,
                 target_yaw_seq, pitch_seq, fire_seq, integral_enable);
}
