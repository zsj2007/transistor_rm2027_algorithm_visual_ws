#include "EKF/alliance_njust/AllianceLightbarPredictor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <Eigen/Cholesky>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi * 0.5;
constexpr double kTwoPi = kPi * 2.0;
constexpr double kSmallArmorWidthM = 0.135;
constexpr double kLargeArmorWidthM = 0.230;
constexpr double kLightbarHeightM = 0.056;

template <class T>
T yamlOr(const YAML::Node& node, const char* key, const T& fallback)
{
    if (node && node[key]) {
        try {
            return node[key].as<T>();
        } catch (...) {
        }
    }
    return fallback;
}

Eigen::Vector3d radialFromProjectYaw(double yaw)
{
    return Eigen::Vector3d(std::sin(yaw), -std::cos(yaw), 0.0);
}

Eigen::Vector3d tangentFromProjectYaw(double yaw)
{
    return Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
}

}  // namespace

AllianceLightbarPredictor::AllianceLightbarPredictor(
    std::shared_ptr<YAML::Node> config_file_ptr,
    std::shared_ptr<RestFrame> rest_frame,
    ArmorType::ArmorType armor_class)
    : rest_frame_(std::move(rest_frame))
{
    (void)armor_class;
    if (!config_file_ptr) {
        throw std::invalid_argument("AllianceLightbarPredictor: null config");
    }
    configure(*config_file_ptr);
    resetCovariance();
    debug_.tracker_state = "LOST";
    debug_.tracker_state_before = "LOST";
}

void AllianceLightbarPredictor::configure(const YAML::Node& root)
{
    const YAML::Node cfg = root["alliance_robot_ekf"];

    config_.radius_forward_min = yamlOr(cfg, "radius_forward_min", config_.radius_forward_min);
    config_.radius_forward_max = yamlOr(cfg, "radius_forward_max", config_.radius_forward_max);
    config_.radius_lateral_min = yamlOr(cfg, "radius_lateral_min", config_.radius_lateral_min);
    config_.radius_lateral_max = yamlOr(cfg, "radius_lateral_max", config_.radius_lateral_max);
    config_.height_lateral_min = yamlOr(cfg, "height_lateral_min", config_.height_lateral_min);
    config_.height_lateral_max = yamlOr(cfg, "height_lateral_max", config_.height_lateral_max);

    config_.noise_x = yamlOr(cfg, "noise_x", config_.noise_x);
    config_.noise_y = yamlOr(cfg, "noise_y", config_.noise_y);
    config_.noise_z = yamlOr(cfg, "noise_z", config_.noise_z);
    config_.noise_vx = yamlOr(cfg, "noise_vx", config_.noise_vx);
    config_.noise_vy = yamlOr(cfg, "noise_vy", config_.noise_vy);
    config_.noise_vz = yamlOr(cfg, "noise_vz", config_.noise_vz);
    config_.noise_yaw = yamlOr(cfg, "noise_rotation_angle", config_.noise_yaw);
    config_.noise_w = yamlOr(cfg, "noise_rotation_speed", config_.noise_w);
    config_.noise_rf = yamlOr(cfg, "noise_radius_forward", config_.noise_rf);
    config_.noise_rl = yamlOr(cfg, "noise_radius_lateral", config_.noise_rl);
    config_.noise_hl = yamlOr(cfg, "noise_height_lateral", config_.noise_hl);
    config_.observation_pixel_variance =
        yamlOr(cfg, "noise_observation", config_.observation_pixel_variance);

    const double yaw_full_deg = yamlOr(cfg, "yaw_full_max_degree", 70.0);
    const double yaw_part_deg = yamlOr(cfg, "yaw_part_max_degree", 90.0);
    const double pitch_deg = yamlOr(cfg, "armor_pitch_degree", 15.0);
    config_.yaw_full_max_rad = yaw_full_deg * kPi / 180.0;
    config_.yaw_part_max_rad = yaw_part_deg * kPi / 180.0;
    config_.armor_pitch_rad = pitch_deg * kPi / 180.0;
    config_.initial_radius_m = yamlOr(cfg, "initial_radius_m", config_.initial_radius_m);
    config_.max_dt_s = yamlOr(cfg, "max_dt_s", config_.max_dt_s);
    config_.min_update_count = yamlOr(cfg, "min_update_count", config_.min_update_count);
    config_.converge_cov_xy = yamlOr(cfg, "converge_cov_xy", config_.converge_cov_xy);

    const YAML::Node K = root["camera_matrix"];
    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
        K[0][0].as<double>(), K[0][1].as<double>(), K[0][2].as<double>(),
        K[1][0].as<double>(), K[1][1].as<double>(), K[1][2].as<double>(),
        K[2][0].as<double>(), K[2][1].as<double>(), K[2][2].as<double>());
    fx_ = camera_matrix_.at<double>(0, 0);
    fy_ = camera_matrix_.at<double>(1, 1);

    const YAML::Node D = root["dist_coeffs"];
    dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
        D[0].as<double>(), D[1].as<double>(), D[2].as<double>(),
        D[3].as<double>(), D[4].as<double>());

    delta_x_m_ = yamlOr(root, "delta_x_", 0.0) / 1000.0;
    delta_y_m_ = yamlOr(root, "delta_y_", 0.0) / 1000.0;
    delta_z_m_ = yamlOr(root, "delta_z_", 0.0) / 1000.0;

    Q_.setZero();
    Q_.diagonal() << config_.noise_x, config_.noise_y, config_.noise_z,
        config_.noise_vx, config_.noise_vy, config_.noise_vz,
        config_.noise_yaw, config_.noise_w,
        config_.noise_rf, config_.noise_rl, config_.noise_hl;
    R_.setIdentity();
    R_ *= config_.observation_pixel_variance;
}

void AllianceLightbarPredictor::resetCovariance()
{
    StateVector diag;
    diag << 64.0, 64.0, 64.0,
        64.0, 64.0, 64.0,
        0.4, 100.0,
        1e-2, 1e-2, 1e-2;
    P_ = diag.asDiagonal();
}

Eigen::Matrix3d AllianceLightbarPredictor::worldFromGunRotation() const
{
    const std::vector<float> rpy = rest_frame_->getCamOrientation();
    const double yaw = rpy.size() > 0 ? rpy[0] : 0.0;
    const double pitch = rpy.size() > 1 ? rpy[1] : 0.0;
    const double roll = rpy.size() > 2 ? rpy[2] : 0.0;

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);

    Eigen::Matrix3d R;
    R << cy * cr - sy * sp * sr, -sy * cp, cy * sr + sy * sp * cr,
         sy * cr + cy * sp * sr,  cy * cp, sy * sr - cy * sp * cr,
         -cp * sr,                  sp,      cp * cr;
    return R;
}

Eigen::Vector3d AllianceLightbarPredictor::cameraPositionWorld() const
{
    const Eigen::Vector3d camera_in_gun(delta_x_m_, delta_y_m_, delta_z_m_);
    return worldFromGunRotation() * camera_in_gun;
}

Eigen::Vector3d AllianceLightbarPredictor::worldToCameraPnp(
    const Eigen::Vector3d& world_point) const
{
    const Eigen::Matrix3d R_world_gun = worldFromGunRotation();
    const Eigen::Vector3d camera_world = cameraPositionWorld();
    const Eigen::Vector3d p_normal = R_world_gun.transpose() * (world_point - camera_world);
    return Eigen::Vector3d(p_normal.x(), -p_normal.z(), p_normal.y());
}

std::optional<Eigen::Vector2d> AllianceLightbarPredictor::projectWorldPoint(
    const Eigen::Vector3d& world_point) const
{
    const Eigen::Vector3d p = worldToCameraPnp(world_point);
    if (!p.allFinite() || p.z() <= 0.05) return std::nullopt;

    std::vector<cv::Point3d> object_points = {
        cv::Point3d(p.x(), p.y(), p.z()),
    };
    std::vector<cv::Point2d> image_points;
    cv::projectPoints(object_points, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0),
                      camera_matrix_, dist_coeffs_, image_points);
    if (image_points.empty() || !std::isfinite(image_points[0].x) ||
        !std::isfinite(image_points[0].y)) {
        return std::nullopt;
    }
    return Eigen::Vector2d(image_points[0].x, image_points[0].y);
}

std::array<Eigen::Vector3d, 4> AllianceLightbarPredictor::armorRadials(
    const StateVector& state) const
{
    std::array<Eigen::Vector3d, 4> result;
    for (int i = 0; i < 4; ++i) {
        result[static_cast<std::size_t>(i)] =
            radialFromProjectYaw(state[YAW] + i * kHalfPi);
    }
    return result;
}

std::array<Eigen::Vector3d, 4> AllianceLightbarPredictor::armorCenters(
    const StateVector& state) const
{
    std::array<Eigen::Vector3d, 4> result;
    const Eigen::Vector3d center(state[X], state[Y], state[Z]);
    const auto radials = armorRadials(state);
    for (int i = 0; i < 4; ++i) {
        const bool lateral = (i == 1 || i == 3);
        const double radius = lateral ? state[RL] : state[RF];
        result[static_cast<std::size_t>(i)] =
            center + radius * radials[static_cast<std::size_t>(i)] +
            (lateral ? state[HL] : 0.0) * Eigen::Vector3d::UnitZ();
    }
    return result;
}

AllianceLightbarPredictor::LightbarGeometry
AllianceLightbarPredictor::lightbarGeometry(const StateVector& state, int lightbar_id) const
{
    LightbarGeometry result;
    if (lightbar_id < 0 || lightbar_id >= 8) return result;

    const int armor_id = lightbar_id / 2;
    const bool right_bar = (lightbar_id % 2) == 1;
    const double angle = state[YAW] + armor_id * kHalfPi;
    const Eigen::Vector3d radial = radialFromProjectYaw(angle);
    const Eigen::Vector3d tangent = tangentFromProjectYaw(angle);
    const auto centers = armorCenters(state);

    const double half_w = (large_armor_ ? kLargeArmorWidthM : kSmallArmorWidthM) * 0.5;
    const double half_h = kLightbarHeightM * 0.5;
    const double cp = std::cos(config_.armor_pitch_rad);
    const double sp = std::sin(config_.armor_pitch_rad);
    const Eigen::Vector3d z_axis = cp * Eigen::Vector3d::UnitZ() - sp * radial;
    const Eigen::Vector3d bar_center =
        centers[static_cast<std::size_t>(armor_id)] +
        (right_bar ? +half_w : -half_w) * tangent;

    result.upper = bar_center + half_h * z_axis;
    result.lower = bar_center - half_h * z_axis;
    return result;
}

void AllianceLightbarPredictor::refreshObservable()
{
    observable_.visible.fill(false);
    const auto centers = armorCenters(x_);
    const auto radials = armorRadials(x_);
    const Eigen::Vector3d cam = cameraPositionWorld();

    for (int i = 0; i < 8; ++i) {
        observable_.world[static_cast<std::size_t>(i)] = lightbarGeometry(x_, i);
        const auto up = projectWorldPoint(observable_.world[static_cast<std::size_t>(i)].upper);
        const auto lo = projectWorldPoint(observable_.world[static_cast<std::size_t>(i)].lower);
        observable_.upper2d[static_cast<std::size_t>(i)] =
            up.value_or(Eigen::Vector2d::Zero());
        observable_.lower2d[static_cast<std::size_t>(i)] =
            lo.value_or(Eigen::Vector2d::Zero());
    }

    for (int armor_id = 0; armor_id < 4; ++armor_id) {
        const Eigen::Vector3d to_cam_raw = cam - centers[static_cast<std::size_t>(armor_id)];
        if (to_cam_raw.norm() < 1e-9) continue;
        const Eigen::Vector3d armor_to_cam = to_cam_raw.normalized();
        const double cosine = std::clamp(
            radials[static_cast<std::size_t>(armor_id)].dot(armor_to_cam), -1.0, 1.0);
        const double facing = std::acos(cosine);
        if (facing >= config_.yaw_part_max_rad) continue;

        const int left_id = armor_id * 2;
        const int right_id = left_id + 1;
        if (facing < config_.yaw_full_max_rad) {
            observable_.visible[static_cast<std::size_t>(left_id)] = true;
            observable_.visible[static_cast<std::size_t>(right_id)] = true;
        } else {
            const double angle = x_[YAW] + armor_id * kHalfPi;
            const Eigen::Vector3d tangent = tangentFromProjectYaw(angle);
            const bool on_plus_t = armor_to_cam.dot(tangent) > 0.0;
            observable_.visible[static_cast<std::size_t>(on_plus_t ? right_id : left_id)] = true;
        }
    }
}

bool AllianceLightbarPredictor::finitePoint(const cv::Point2f& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y);
}

bool AllianceLightbarPredictor::validPixelArmor(const ArmorResult& armor)
{
    const auto& pts = armor.armor.raw_light_bar_endpoints;
    return std::all_of(pts.begin(), pts.end(), finitePoint);
}

std::array<AllianceLightbarPredictor::ObservedBar, 2>
AllianceLightbarPredictor::observedBars(const ArmorResult& armor)
{
    const auto& p = armor.armor.raw_light_bar_endpoints;
    return {
        ObservedBar { Eigen::Vector2d(p[0].x, p[0].y), Eigen::Vector2d(p[1].x, p[1].y) },
        ObservedBar { Eigen::Vector2d(p[3].x, p[3].y), Eigen::Vector2d(p[2].x, p[2].y) },
    };
}

double AllianceLightbarPredictor::squaredDistance(
    const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
    return (a - b).squaredNorm();
}

double AllianceLightbarPredictor::wrapAngle(double angle)
{
    while (angle > kPi) angle -= kTwoPi;
    while (angle <= -kPi) angle += kTwoPi;
    return angle;
}

bool AllianceLightbarPredictor::initializeFromPnp(
    const std::vector<ArmorResult>& armors)
{
    const ArmorResult* best = nullptr;
    double best_width = -1.0;
    for (const ArmorResult& armor : armors) {
        if (!validPixelArmor(armor) || !armor.solve_armor_result.valid) continue;
        const auto bars = observedBars(armor);
        const double width = (bars[1].upper - bars[0].upper).norm();
        if (width > best_width) {
            best_width = width;
            best = &armor;
        }
    }
    if (!best) return false;

    const AimResult& pnp = best->solve_armor_result;
    const cv::Point3f world_mm = rest_frame_->pnpToWorldP3f(pnp.position);
    if (!std::isfinite(world_mm.x) || !std::isfinite(world_mm.y) || !std::isfinite(world_mm.z)) {
        return false;
    }

    const std::vector<float> world_euler = rest_frame_->getWorldEulerAnglesFromCam(
        static_cast<float>(pnp.normal_euler_angles.size() > 0 ? pnp.normal_euler_angles[0] : 0.0),
        static_cast<float>(pnp.normal_euler_angles.size() > 1 ? pnp.normal_euler_angles[1] : 0.0),
        static_cast<float>(pnp.normal_euler_angles.size() > 2 ? pnp.normal_euler_angles[2] : 0.0));
    double yaw = world_euler.empty() ? 0.0 : world_euler[0];
    yaw = wrapAngle(yaw);

    const Eigen::Vector3d armor_pos(
        world_mm.x / 1000.0, world_mm.y / 1000.0, world_mm.z / 1000.0);

    auto make_seed = [&](double seed_yaw) {
        StateVector seed = StateVector::Zero();
        const Eigen::Vector3d radial = radialFromProjectYaw(seed_yaw);
        const Eigen::Vector3d center = armor_pos - config_.initial_radius_m * radial;
        seed[X] = center.x();
        seed[Y] = center.y();
        seed[Z] = center.z();
        seed[YAW] = seed_yaw;
        seed[RF] = config_.initial_radius_m;
        seed[RL] = config_.initial_radius_m;
        seed[HL] = 0.0;
        return seed;
    };

    large_armor_ = best->is_large;

    auto visible_count_for = [&](const StateVector& seed) {
        const StateVector saved = x_;
        const bool saved_initialized = initialized_;
        x_ = seed;
        initialized_ = true;
        refreshObservable();
        int count = 0;
        for (int id = 0; id < 4; ++id) {
            if (observable_.visible[static_cast<std::size_t>(id * 2)] &&
                observable_.visible[static_cast<std::size_t>(id * 2 + 1)]) {
                ++count;
            }
        }
        x_ = saved;
        initialized_ = saved_initialized;
        return count;
    };

    StateVector seed = make_seed(yaw);
    StateVector alt = make_seed(wrapAngle(yaw + kPi));
    if (visible_count_for(alt) > visible_count_for(seed)) {
        seed = alt;
    }

    x_ = seed;
    resetCovariance();
    initialized_ = true;
    update_count_ = 0;
    update_frames_ = 1;
    lost_frames_ = 0;
    last_matched_armor_id_ = 0;
    debug_flip_flag_ = 1;
    refreshObservable();

    debug_ = EKFTargetDebugState{};
    debug_.tracker_state = "DETECTING";
    debug_.tracker_state_before = "LOST";
    debug_.geometry_valid = true;
    debug_.current_armor_id = 0;
    debug_.matched_id = 0;
    debug_.best_id = 0;
    debug_.measurement_valid = true;

    debug_.measurement << armor_pos.x(), armor_pos.y(), armor_pos.z(), x_[YAW];
    debug_.measurement_yaw = x_[YAW];
    return true;
}

double AllianceLightbarPredictor::consumeDt(double update_time)
{
    debug_.time_discontinuity = false;
    if (!std::isfinite(update_time)) {
        debug_.time_discontinuity = true;
        return -1.0;
    }
    if (!has_update_time_) {
        last_update_time_ = update_time;
        has_update_time_ = true;
        last_dt_s_ = 0.0;
        return 0.0;
    }
    const double raw = update_time - last_update_time_;
    if (!std::isfinite(raw) || raw <= 0.0) {
        debug_.time_discontinuity = true;
        return -1.0;
    }
    last_update_time_ = update_time;
    last_dt_s_ = std::min(raw, std::max(1e-4, config_.max_dt_s));
    if (raw > config_.max_dt_s) debug_.time_discontinuity = true;
    return last_dt_s_;
}

void AllianceLightbarPredictor::predictStep(double dt)
{
    if (!initialized_ || dt <= 0.0) return;

    x_[X] += x_[VX] * dt;
    x_[Y] += x_[VY] * dt;
    x_[Z] += x_[VZ] * dt;
    x_[YAW] = wrapAngle(x_[YAW] + x_[W] * dt);

    Covariance F = Covariance::Identity();
    F(X, VX) = dt;
    F(Y, VY) = dt;
    F(Z, VZ) = dt;
    F(YAW, W) = dt;
    P_ = F * P_ * F.transpose() + Q_;
    P_ = 0.5 * (P_ + P_.transpose());
    refreshObservable();
}

AllianceLightbarPredictor::ObservationJacobian
AllianceLightbarPredictor::makeObservationJacobian(int lightbar_id) const
{
    ObservationJacobian H = ObservationJacobian::Zero();
    if (lightbar_id < 0 || lightbar_id >= 8 || !x_.allFinite()) return H;

    const Eigen::Matrix3d R_world_gun = worldFromGunRotation();
    Eigen::Matrix3d normal_to_pnp;
    normal_to_pnp << 1.0, 0.0, 0.0,
                     0.0, 0.0, -1.0,
                     0.0, 1.0, 0.0;
    const Eigen::Matrix3d J_camera_world = normal_to_pnp * R_world_gun.transpose();

    constexpr double epsilon = 1e-4;
    auto yaw_derivative = [&](bool upper) {
        StateVector plus = x_;
        StateVector minus = x_;
        plus[YAW] = wrapAngle(plus[YAW] + epsilon);
        minus[YAW] = wrapAngle(minus[YAW] - epsilon);
        const LightbarGeometry gp = lightbarGeometry(plus, lightbar_id);
        const LightbarGeometry gm = lightbarGeometry(minus, lightbar_id);
        return ((upper ? gp.upper : gp.lower) - (upper ? gm.upper : gm.lower)) /
               (2.0 * epsilon);
    };

    const int armor_id = lightbar_id / 2;
    const double armor_angle = x_[YAW] + armor_id * kHalfPi;
    const Eigen::Vector3d radial = radialFromProjectYaw(armor_angle);

    for (int endpoint = 0; endpoint < 2; ++endpoint) {
        const bool upper = endpoint == 0;
        const int row = endpoint * 2;
        const Eigen::Vector3d world_point = upper
            ? observable_.world[static_cast<std::size_t>(lightbar_id)].upper
            : observable_.world[static_cast<std::size_t>(lightbar_id)].lower;
        const Eigen::Vector3d camera_point = worldToCameraPnp(world_point);
        const double depth = camera_point.z();
        if (!camera_point.allFinite() || depth < 0.10) continue;

        Eigen::Matrix<double, 2, 3> J_pixel_camera;
        J_pixel_camera << fx_ / depth, 0.0,
            -fx_ * camera_point.x() / (depth * depth),
            0.0, fy_ / depth,
            -fy_ * camera_point.y() / (depth * depth);

        Eigen::Matrix<double, 3, kStateDim> J_geometry =
            Eigen::Matrix<double, 3, kStateDim>::Zero();
        J_geometry(0, X) = 1.0;
        J_geometry(1, Y) = 1.0;
        J_geometry(2, Z) = 1.0;
        J_geometry.block<3, 1>(0, YAW) = yaw_derivative(upper);

        if (armor_id == 0 || armor_id == 2) {
            J_geometry.block<3, 1>(0, RF) = radial;
        } else {
            J_geometry.block<3, 1>(0, RL) = radial;
            J_geometry(2, HL) = 1.0;
        }

        H.block<2, kStateDim>(row, 0).noalias() =
            J_pixel_camera * J_camera_world * J_geometry;
    }
    return H;
}

bool AllianceLightbarPredictor::updateOneBar(
    const ObservedBar& bar, int lightbar_id)
{
    if (lightbar_id < 0 || lightbar_id >= 8) return false;
    const Eigen::Vector2d pred_upper = observable_.upper2d[static_cast<std::size_t>(lightbar_id)];
    const Eigen::Vector2d pred_lower = observable_.lower2d[static_cast<std::size_t>(lightbar_id)];
    if (!pred_upper.allFinite() || !pred_lower.allFinite()) return false;

    Eigen::Vector4d innovation;
    innovation << bar.upper.x() - pred_upper.x(), bar.upper.y() - pred_upper.y(),
                  bar.lower.x() - pred_lower.x(), bar.lower.y() - pred_lower.y();

    const ObservationJacobian H = makeObservationJacobian(lightbar_id);
    if (!H.allFinite() || H.squaredNorm() < 1e-12) return false;

    const Eigen::Matrix4d S = H * P_ * H.transpose() + R_;
    const Eigen::LDLT<Eigen::Matrix4d> ldlt(S);
    if (ldlt.info() != Eigen::Success) return false;

    const Eigen::Matrix4d S_inv = ldlt.solve(Eigen::Matrix4d::Identity());
    if (!S_inv.allFinite()) return false;
    const KalmanGain K = P_ * H.transpose() * S_inv;

    StateVector posterior = x_ + K * innovation;
    if (!posterior.allFinite()) return false;
    posterior[YAW] = wrapAngle(posterior[YAW]);
    posterior[RF] = std::clamp(posterior[RF],
        config_.radius_forward_min, config_.radius_forward_max);
    posterior[RL] = std::clamp(posterior[RL],
        config_.radius_lateral_min, config_.radius_lateral_max);
    posterior[HL] = std::clamp(posterior[HL],
        config_.height_lateral_min, config_.height_lateral_max);
    posterior[VX] = std::clamp(posterior[VX], -10.0, 10.0);
    posterior[VY] = std::clamp(posterior[VY], -10.0, 10.0);

    const Covariance I_KH = Covariance::Identity() - K * H;
    Covariance posterior_cov = I_KH * P_ * I_KH.transpose();
    posterior_cov += K * R_ * K.transpose();
    posterior_cov = 0.5 * (posterior_cov + posterior_cov.transpose());
    if (!posterior_cov.allFinite()) return false;

    x_ = posterior;
    P_ = posterior_cov;
    debug_.nis = innovation.dot(S_inv * innovation);
    refreshObservable();
    return true;
}

bool AllianceLightbarPredictor::correct(const std::vector<ArmorResult>& armors)
{
    struct Anchor {
        const ArmorResult* armor = nullptr;
        int armor_id = -1;
        double error = std::numeric_limits<double>::max();
    } anchor;

    refreshObservable();

    for (const ArmorResult& armor : armors) {
        if (!validPixelArmor(armor)) continue;
        const auto obs = observedBars(armor);
        for (int armor_id = 0; armor_id < 4; ++armor_id) {
            const int left_id = armor_id * 2;
            const int right_id = left_id + 1;
            if (!observable_.visible[static_cast<std::size_t>(left_id)] ||
                !observable_.visible[static_cast<std::size_t>(right_id)]) {
                continue;
            }

            const double error =
                squaredDistance(obs[0].upper, observable_.upper2d[static_cast<std::size_t>(left_id)]) +
                squaredDistance(obs[0].lower, observable_.lower2d[static_cast<std::size_t>(left_id)]) +
                squaredDistance(obs[1].upper, observable_.upper2d[static_cast<std::size_t>(right_id)]) +
                squaredDistance(obs[1].lower, observable_.lower2d[static_cast<std::size_t>(right_id)]);
            if (error < anchor.error) {
                anchor = Anchor { &armor, armor_id, error };
            }
        }
    }
    if (!anchor.armor) return false;

    struct SortedEntry {
        int assigned_id = -1;
        ObservedBar bar;
    };
    std::vector<SortedEntry> sorted;
    for (const ArmorResult& armor : armors) {
        if (!validPixelArmor(armor)) continue;
        const auto bars = observedBars(armor);
        sorted.push_back({ -1, bars[0] });
        sorted.push_back({ -1, bars[1] });
    }
    if (sorted.empty()) return false;
    std::sort(sorted.begin(), sorted.end(), [](const SortedEntry& a, const SortedEntry& b) {
        return a.bar.upper.x() < b.bar.upper.x();
    });

    const auto anchor_bars = observedBars(*anchor.armor);
    auto closest_index = [&](const Eigen::Vector2d& point) {
        std::size_t best = 0;
        double dist = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < sorted.size(); ++i) {
            const double d = std::abs(sorted[i].bar.upper.x() - point.x());
            if (d < dist) {
                dist = d;
                best = i;
            }
        }
        return best;
    };

    const std::size_t anchor_left = closest_index(anchor_bars[0].upper);
    const std::size_t anchor_right = closest_index(anchor_bars[1].upper);
    if (anchor_left == anchor_right) return false;

    sorted[anchor_left].assigned_id = anchor.armor_id * 2;
    sorted[anchor_right].assigned_id = anchor.armor_id * 2 + 1;

    for (std::size_t i = anchor_left + 1; i < sorted.size(); ++i) {
        sorted[i].assigned_id = (sorted[i - 1].assigned_id + 1) % 8;
    }
    for (std::size_t i = anchor_left; i > 0; --i) {
        sorted[i - 1].assigned_id = (sorted[i].assigned_id - 1 + 8) % 8;
    }

    const bool switched =
        last_matched_armor_id_ >= 0 && last_matched_armor_id_ != anchor.armor_id;
    const int previous_id = last_matched_armor_id_;

    const auto pre_centers = armorCenters(x_);
    const double pre_yaw = wrapAngle(x_[YAW] + anchor.armor_id * kHalfPi);
    debug_.pre_predicted <<
        pre_centers[static_cast<std::size_t>(anchor.armor_id)].x(),
        pre_centers[static_cast<std::size_t>(anchor.armor_id)].y(),
        pre_centers[static_cast<std::size_t>(anchor.armor_id)].z(), pre_yaw;

    bool any_update = false;
    for (const SortedEntry& entry : sorted) {
        if (entry.assigned_id < 0) continue;
        any_update = updateOneBar(entry.bar, entry.assigned_id) || any_update;
    }
    if (!any_update) return false;

    ++update_count_;
    ++update_frames_;
    lost_frames_ = 0;
    last_matched_armor_id_ = anchor.armor_id;
    debug_flip_flag_ = (anchor.armor_id % 2) + 1;

    const auto post_centers = armorCenters(x_);
    const double post_yaw = wrapAngle(x_[YAW] + anchor.armor_id * kHalfPi);
    debug_.post_predicted <<
        post_centers[static_cast<std::size_t>(anchor.armor_id)].x(),
        post_centers[static_cast<std::size_t>(anchor.armor_id)].y(),
        post_centers[static_cast<std::size_t>(anchor.armor_id)].z(), post_yaw;

    if (anchor.armor->solve_armor_result.valid) {
        const AimResult& pnp = anchor.armor->solve_armor_result;
        const cv::Point3f p = rest_frame_->pnpToWorldP3f(pnp.position);
        const auto e = rest_frame_->getWorldEulerAnglesFromCam(
            static_cast<float>(pnp.normal_euler_angles.size() > 0 ? pnp.normal_euler_angles[0] : 0.0),
            static_cast<float>(pnp.normal_euler_angles.size() > 1 ? pnp.normal_euler_angles[1] : 0.0),
            static_cast<float>(pnp.normal_euler_angles.size() > 2 ? pnp.normal_euler_angles[2] : 0.0));
        const double pnp_yaw = e.empty() ? std::numeric_limits<double>::quiet_NaN() : e[0];
        debug_.measurement << p.x / 1000.0, p.y / 1000.0, p.z / 1000.0, pnp_yaw;
        debug_.measurement_yaw = pnp_yaw;
        debug_.predicted_yaw = debug_.pre_predicted[3];
        debug_.yaw_innovation = std::isfinite(pnp_yaw)
            ? wrapAngle(pnp_yaw - debug_.predicted_yaw)
            : std::numeric_limits<double>::quiet_NaN();
        debug_.pre_residual = debug_.measurement.head<3>() - debug_.pre_predicted.head<3>();
        debug_.post_residual = debug_.measurement.head<3>() - debug_.post_predicted.head<3>();
        debug_.pre_position_error = debug_.pre_residual.norm();
        debug_.post_position_error = debug_.post_residual.norm();
    } else {
        debug_.measurement = Eigen::Vector4d::Constant(
            std::numeric_limits<double>::quiet_NaN());
        debug_.measurement_yaw = std::numeric_limits<double>::quiet_NaN();
        debug_.predicted_yaw = debug_.pre_predicted[3];
        debug_.yaw_innovation = std::numeric_limits<double>::quiet_NaN();
        debug_.pre_residual = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
        debug_.post_residual = debug_.pre_residual;
        debug_.pre_position_error = std::numeric_limits<double>::quiet_NaN();
        debug_.post_position_error = std::numeric_limits<double>::quiet_NaN();
    }

    debug_.matched_id = anchor.armor_id;
    debug_.best_id = anchor.armor_id;
    debug_.current_armor_id = anchor.armor_id;
    debug_.armor_parity = anchor.armor_id % 2;
    debug_.armor_switched = switched;
    debug_.candidate_is_switch = switched;
    debug_.topology_event = switched;
    debug_.recovered = previous_id >= 0 && debug_.tracker_state == "TEMP_LOST";
    debug_.measurement_valid = true;
    debug_.updated = true;
    debug_.geometry_update_allowed = true;
    debug_.geometry_preserved = false;
    debug_.geometry_valid = true;
    return true;
}

bool AllianceLightbarPredictor::update(
    const std::vector<ArmorResult>& armors, double update_time)
{
    const std::string before = debug_.tracker_state;
    debug_.tracker_state_before = before;
    debug_.updated = false;
    debug_.measurement_valid = false;
    debug_.geometry_update_allowed = false;
    debug_.armor_switched = false;
    debug_.candidate_is_switch = false;
    debug_.topology_event = false;
    debug_.recovered = false;

    const double dt = consumeDt(update_time);
    debug_.dt_s = dt >= 0.0 ? dt : 0.0;
    if (dt < 0.0) return false;

    if (!initialized_) {
        if (!initializeFromPnp(armors)) {
            debug_.tracker_state = "LOST";
            return false;
        }
    } else {
        predictStep(dt);
    }

    const bool corrected = correct(armors);
    if (corrected) {
        debug_.tracker_state = ready() ? "TRACKING" : "DETECTING";
    } else {
        ++lost_frames_;
        debug_.tracker_state = "TEMP_LOST";
        debug_.geometry_valid = initialized_;
    }
    debug_.lost_frames = lost_frames_;

    if (diverged()) {
        clear();
        debug_.tracker_state_before = before;
        return false;
    }

    debug_.r1_m = x_[RF];
    debug_.r2_m = x_[RL];
    debug_.h_m = x_[HL];
    debug_.p_r1_m2 = P_(RF, RF);
    debug_.p_r2_m2 = P_(RL, RL);
    debug_.p_h_m2 = P_(HL, HL);
    debug_.p_x_m2 = P_(X, X);
    debug_.p_vx_m2_s2 = P_(VX, VX);
    debug_.p_y_m2 = P_(Y, Y);
    debug_.p_vy_m2_s2 = P_(VY, VY);
    return corrected;
}

void AllianceLightbarPredictor::missUpdate(double update_time)
{
    const std::string before = debug_.tracker_state;
    debug_.tracker_state_before = before;
    debug_.updated = false;
    debug_.measurement_valid = false;
    debug_.geometry_update_allowed = false;
    debug_.armor_switched = false;

    const double dt = consumeDt(update_time);
    debug_.dt_s = dt >= 0.0 ? dt : 0.0;
    if (dt < 0.0 || !initialized_) return;
    predictStep(dt);
    ++lost_frames_;
    debug_.lost_frames = lost_frames_;
    debug_.tracker_state = "TEMP_LOST";
    debug_.geometry_valid = true;

    if (diverged()) clear();
}

void AllianceLightbarPredictor::clear()
{
    initialized_ = false;
    has_update_time_ = false;
    last_update_time_ = 0.0;
    last_dt_s_ = 0.0;
    update_frames_ = 0;
    update_count_ = 0;
    lost_frames_ = 0;
    last_matched_armor_id_ = -1;
    debug_flip_flag_ = 1;
    x_.setZero();
    resetCovariance();
    observable_.visible.fill(false);
    debug_ = EKFTargetDebugState{};
    debug_.tracker_state = "LOST";
    debug_.tracker_state_before = "LOST";
}

bool AllianceLightbarPredictor::diverged() const
{
    if (!initialized_) return false;
    if (!x_.allFinite() || !P_.allFinite()) return true;
    return P_(X, X) > 150.0 || P_(Y, Y) > 150.0 ||
           std::abs(x_[X]) > 15.0 || std::abs(x_[Y]) > 15.0 ||
           std::abs(x_[Z]) > 2.0 || std::abs(x_[W]) > 10.0 * kPi ||
           std::abs(x_[VX]) > 5.0 || std::abs(x_[VY]) > 5.0 ||
           std::abs(x_[VZ]) > 1.0;
}

bool AllianceLightbarPredictor::ready() const
{
    if (!initialized_) return false;
    if (lost_frames_ != 0) return false;
    if (update_count_ < config_.min_update_count) return false;
    if (P_(X, X) > config_.converge_cov_xy || P_(Y, Y) > config_.converge_cov_xy) {
        return false;
    }
    return !diverged();
}

bool AllianceLightbarPredictor::hasState() const
{
    return initialized_ && !diverged();
}

EKFTargetPrediction AllianceLightbarPredictor::predict(double predict_time) const
{
    EKFTargetPrediction result;
    if (!hasState()) return result;

    StateVector state = x_;
    state[X] += state[VX] * predict_time;
    state[Y] += state[VY] * predict_time;
    state[Z] += state[VZ] * predict_time;
    state[YAW] = wrapAngle(state[YAW] + state[W] * predict_time);

    result.center_x = state[X] * 1000.0;
    result.center_y = state[Y] * 1000.0;
    result.center_z = state[Z] * 1000.0;
    result.alternate_z = (state[Z] + state[HL]) * 1000.0;
    result.r1 = state[RF] * 1000.0;
    result.r2 = state[RL] * 1000.0;
    result.h = state[HL] * 1000.0;
    result.yaw = state[YAW];
    result.w = state[W];
    result.rotation_direction = state[W] >= 0.0 ? 1 : -1;

    const Eigen::Vector3d center(state[X], state[Y], state[Z]);
    result.armors.reserve(4);
    for (int id = 0; id < 4; ++id) {
        const double yaw = wrapAngle(state[YAW] + id * kHalfPi);
        const bool lateral = id == 1 || id == 3;
        const double radius = lateral ? state[RL] : state[RF];
        const Eigen::Vector3d p = center + radius * radialFromProjectYaw(yaw) +
            (lateral ? state[HL] : 0.0) * Eigen::Vector3d::UnitZ();
        result.armors.push_back(EKFPredictedArmor {
            p.x() * 1000.0,
            p.y() * 1000.0,
            p.z() * 1000.0,
            radius * 1000.0,
            yaw,
        });
    }
    return result;
}

EKFTargetState AllianceLightbarPredictor::state() const
{
    EKFTargetState result;
    if (!hasState()) return result;
    result.center_x = x_[X] * 1000.0;
    result.center_y = x_[Y] * 1000.0;
    result.center_z = x_[Z] * 1000.0;
    result.center_vx = x_[VX] * 1000.0;
    result.center_vy = x_[VY] * 1000.0;
    result.center_vz = x_[VZ] * 1000.0;
    result.r1 = x_[RF] * 1000.0;
    result.r2 = x_[RL] * 1000.0;
    result.h = x_[HL] * 1000.0;
    result.yaw = x_[YAW];
    result.w = x_[W];
    result.update_frames = update_frames_;
    return result;
}

EKFTargetDebugState AllianceLightbarPredictor::debugState() const
{
    EKFTargetDebugState out = debug_;
    if (initialized_) {
        out.r1_m = x_[RF];
        out.r2_m = x_[RL];
        out.h_m = x_[HL];
        out.p_r1_m2 = P_(RF, RF);
        out.p_r2_m2 = P_(RL, RL);
        out.p_h_m2 = P_(HL, HL);
        out.p_x_m2 = P_(X, X);
        out.p_vx_m2_s2 = P_(VX, VX);
        out.p_y_m2 = P_(Y, Y);
        out.p_vy_m2_s2 = P_(VY, VY);
        out.geometry_valid = true;
    }
    return out;
}

std::vector<AllianceLightbarPredictor::ProjectedLightbar>
AllianceLightbarPredictor::projectedLightbars() const
{
    std::vector<ProjectedLightbar> result;
    if (!hasState()) return result;
    result.reserve(8);
    for (int id = 0; id < 8; ++id) {
        const auto& up = observable_.upper2d[static_cast<std::size_t>(id)];
        const auto& lo = observable_.lower2d[static_cast<std::size_t>(id)];
        if (!up.allFinite() || !lo.allFinite()) continue;
        result.push_back(ProjectedLightbar {
            id,
            cv::Point2f(static_cast<float>(up.x()), static_cast<float>(up.y())),
            cv::Point2f(static_cast<float>(lo.x()), static_cast<float>(lo.y())),
            observable_.visible[static_cast<std::size_t>(id)],
        });
    }
    return result;
}
