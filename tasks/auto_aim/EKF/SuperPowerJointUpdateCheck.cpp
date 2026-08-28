#include "EKF/SuperPowerPredictor.h"
#include "EKF/SuperPowerTarget.h"
#include "utils/DataProcessFuncs.h"

#include <cmath>
#include <iostream>
#include <string>

namespace {
constexpr double kPi = 3.14159265358979323846;

Eigen::VectorXd initialCovariance() {
    Eigen::VectorXd diag(11);
    diag << 1.0, 64.0,
            1.0, 64.0,
            1.0, 64.0,
            0.4, 100.0,
            1.0, 1.0, 1.0;
    return diag;
}

sp_ekf::ArmorObservation observation(
    double x, double y, double z, double angle) {
    sp_ekf::ArmorObservation value;
    value.xyz = Eigen::Vector3d(x, y, z);
    value.angle = angle;
    return value;
}

bool expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << std::endl;
        return false;
    }
    return true;
}

bool adjacentCandidateTest(double secondary_y,
                           double secondary_angle,
                           int expected_id) {
    const auto primary = observation(1.0, 0.0, 0.0, 0.0);
    sp_ekf::Target target(primary, 0.2, 4, initialCovariance());
    target.predict(0.01);

    sp_ekf::PairUpdateConfig config;
    config.enabled = true;
    config.max_joint_nis = 1e6;
    config.max_secondary_position_error_m = 0.5;
    config.max_secondary_angle_error_rad = 0.5;
    config.measurement_variance_scale = 1.0;
    config.angle_variance_scale = 1.0;

    const auto secondary =
        observation(1.2, secondary_y, 0.0, secondary_angle);
    const auto debug = target.updatePair(primary, secondary, config);

    bool ok = true;
    ok &= expect(debug.pair_used, "valid adjacent pair was rejected");
    ok &= expect(debug.second_matched_id == expected_id,
                 "secondary armor matched wrong adjacent topology");
    ok &= expect(debug.pair_status == "JOINT_OK",
                 "valid pair did not report JOINT_OK");
    ok &= expect(std::isfinite(debug.joint_nis),
                 "joint NIS is not finite");

    const Eigen::VectorXd x = target.ekfX();
    const double updated_r2 = x[8] + x[9];
    ok &= expect(std::abs(updated_r2 - std::abs(secondary_y)) <
                     std::abs(0.2 - std::abs(secondary_y)),
                 "joint observation did not move r2 toward the second PnP");
    return ok;
}

bool fallbackTest() {
    const auto primary = observation(1.0, 0.0, 0.0, 0.0);
    sp_ekf::Target target(primary, 0.2, 4, initialCovariance());
    target.predict(0.01);

    sp_ekf::PairUpdateConfig config;
    config.enabled = true;
    config.max_joint_nis = 20.09;
    config.max_secondary_position_error_m = 0.15;
    config.max_secondary_angle_error_rad = 0.3;

    const auto invalid_secondary =
        observation(3.0, 3.0, 0.0, kPi / 2.0);
    const auto debug =
        target.updatePair(primary, invalid_secondary, config);

    bool ok = true;
    ok &= expect(debug.pair_requested,
                 "fallback did not retain pair-request diagnostic");
    ok &= expect(!debug.pair_used,
                 "invalid secondary unexpectedly entered joint update");
    ok &= expect(debug.pair_status == "SECONDARY_GEOMETRY_GATE",
                 "invalid secondary reported wrong fallback reason");
    ok &= expect(debug.matched_id == 0,
                 "fallback did not perform the primary single update");
    return ok;
}

// 验证正对损失的归一化边界，以及切板惩罚的保持与切换条件。
bool armorSelectionLossTest() {
    bool ok = true;
    const cv::Point2d camera_to_center(0.0, 1.0);
    ok &= expect(std::abs(normalizedArmorFacingLoss(
                             camera_to_center, 0.0)) < 1e-12,
                 "front-facing loss is not zero");
    ok &= expect(std::abs(normalizedArmorFacingLoss(
                             camera_to_center, kPi / 2.0) - 0.5) < 1e-12,
                 "side-facing loss is not 0.5");
    ok &= expect(std::abs(normalizedArmorFacingLoss(
                             camera_to_center, kPi) - 1.0) < 1e-12,
                 "back-facing loss is not one");

    ok &= expect(selectArmorByFacingAndSwitchPenalty(
                     {0.10, 0.05}, 0, 0.10) == 0,
                 "switch penalty did not retain the current armor");
    ok &= expect(selectArmorByFacingAndSwitchPenalty(
                     {0.25, 0.05}, 0, 0.10) == 1,
                 "facing gain did not overcome the switch penalty");
    ok &= expect(selectArmorByFacingAndSwitchPenalty(
                     {0.10, 0.05}, -1, 0.10) == 1,
                 "first selection should not pay a switch penalty");
    return ok;
}

// 根据给定车体相位和物理装甲板 ID 生成理想旋转观测，供拟合测试使用。
// 输出位置单位为毫米、偏航角单位为弧度、时间单位为秒。
EKFTargetObservation rotatingObservation(
    double t, double phase, int armor_id) {
    constexpr double center_x_m = 3.0;
    constexpr double center_y_m = 0.0;
    constexpr double radius_m = 0.2;
    const double angle = phase + armor_id * kPi / 2.0;
    return EKFTargetObservation{
        (center_x_m - radius_m * std::cos(angle)) * 1000.0,
        (center_y_m - radius_m * std::sin(angle)) * 1000.0,
        0.0,
        angle - kPi / 2.0,
        t,
    };
}

// 构造跨物理装甲板 ID 的匀速旋转序列，验证相位解包后的最小二乘 w。
// 同时检查拟合值已真正回写到预测器状态，而非只保存在诊断字段中。
bool angularVelocityLeastSquaresTest() {
    auto config = std::make_shared<YAML::Node>(YAML::Load(R"(
superpower_ekf:
  min_detect_count: 1
  max_temp_lost_count: 10
  max_dt_s: 0.1
  initial_radius_m: 0.2
  armor_num: 4
  angular_velocity_fit:
    window_s: 0.20
    min_samples: 4
)"));
    constexpr double expected_w = 2.0;
    constexpr double initial_phase = 0.2;
    SuperPowerPredictor predictor(
        rotatingObservation(0.0, initial_phase, 0), 200.0, config);
    for (int frame = 1; frame <= 10; ++frame) {
        const double t = 0.02 * frame;
        const int armor_id = frame < 6 ? 0 : 1;
        predictor.update(rotatingObservation(
            t, initial_phase + expected_w * t, armor_id));
    }

    const EKFTargetDebugState debug = predictor.debugState();
    bool ok = true;
    ok &= expect(predictor.ready(), "predictor did not enter TRACKING");
    ok &= expect(debug.phase_observer_valid,
                 "least-squares phase observer did not become valid");
    ok &= expect(debug.phase_w_applied,
                 "least-squares angular velocity was not applied");
    ok &= expect(std::abs(predictor.state().w - expected_w) < 0.10,
                 "least-squares angular velocity is inaccurate");
    return ok;
}
}  // namespace

int main() {
    bool ok = true;
    ok &= adjacentCandidateTest(-0.28, kPi / 2.0, 1);
    ok &= adjacentCandidateTest(0.28, -kPi / 2.0, 3);
    ok &= fallbackTest();
    ok &= armorSelectionLossTest();
    ok &= angularVelocityLeastSquaresTest();
    if (ok) {
        std::cout << "SuperPower joint update tests passed" << std::endl;
        return 0;
    }
    return 1;
}
