#include "EKF/SuperPowerTarget.h"

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
}  // namespace

int main() {
    bool ok = true;
    ok &= adjacentCandidateTest(-0.28, kPi / 2.0, 1);
    ok &= adjacentCandidateTest(0.28, -kPi / 2.0, 3);
    ok &= fallbackTest();
    if (ok) {
        std::cout << "SuperPower joint update tests passed" << std::endl;
        return 0;
    }
    return 1;
}
