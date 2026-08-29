#include "EKF/SuperPowerPredictor.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <mutex>

namespace {
std::mutex g_measurement_mutex;
std::vector<ArmorResult> g_measurements;
std::weak_ptr<RestFrame> g_rest_frame;

std::string lowerCopy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}
}

void SuperPowerPredictor::setRuntimeRestFrame(
    const std::shared_ptr<RestFrame>& rest_frame)
{
    std::lock_guard<std::mutex> lock(g_measurement_mutex);
    g_rest_frame = rest_frame;
}

void SuperPowerPredictor::publishFrameMeasurements(
    const std::vector<ArmorResult>& measurements)
{
    std::lock_guard<std::mutex> lock(g_measurement_mutex);
    g_measurements = measurements;
}

bool SuperPowerPredictor::readAllianceBackend(const YAML::Node& root)
{
    std::string value = "superpower";
    const YAML::Node n = root["ordinary_vehicle_ekf"];
    if (n && n["backend"]) {
        value = n["backend"].as<std::string>();
    }
    value = lowerCopy(value);
    if (value == "alliance" || value == "njust" ||
        value == "alliance_njust" || value == "alliance-lightbar") {
        return true;
    }
    if (value == "superpower" || value == "tongji" || value == "sp" ||
        value == "superpower_tongji") {
        return false;
    }
    throw std::invalid_argument(
        "ordinary_vehicle_ekf backend must be superpower or alliance");
}

std::vector<ArmorResult>
SuperPowerPredictor::currentMeasurementsForTarget(int target_number)
{
    std::lock_guard<std::mutex> lock(g_measurement_mutex);
    std::vector<ArmorResult> out;
    for (const ArmorResult& armor : g_measurements) {
        if (target_number < 0 || armor.number == target_number) out.push_back(armor);
    }
    return out;
}

int SuperPowerPredictor::inferTargetNumber(
    const EKFTargetObservation& initial_observation) const
{
    std::shared_ptr<RestFrame> rest;
    std::vector<ArmorResult> measurements;
    {
        std::lock_guard<std::mutex> lock(g_measurement_mutex);
        rest = g_rest_frame.lock();
        measurements = g_measurements;
    }
    if (!rest) return -1;

    int best_number = -1;
    double best_error = std::numeric_limits<double>::infinity();
    for (const ArmorResult& armor : measurements) {
        if (!armor.solve_armor_result.valid) continue;
        const cv::Point3f p = rest->pnpToWorldP3f(armor.solve_armor_result.position);
        const double dx = static_cast<double>(p.x) - initial_observation.x;
        const double dy = static_cast<double>(p.y) - initial_observation.y;
        const double dz = static_cast<double>(p.z) - initial_observation.z;
        const double error = dx*dx + dy*dy + dz*dz;
        if (error < best_error) {
            best_error = error;
            best_number = armor.number;
        }
    }
    return best_number;
}

SuperPowerPredictor::SuperPowerPredictor(
    const EKFTargetObservation& initial_observation,
    double initial_radius_mm,
    std::shared_ptr<YAML::Node> config_file_ptr)
    : config_(std::move(config_file_ptr)),
      initial_radius_mm_(initial_radius_mm)
{
    if (!config_) throw std::invalid_argument("SuperPowerPredictor facade: null config");
    alliance_backend_ = readAllianceBackend(*config_);

    if (!alliance_backend_) {
        superpower_ = std::make_unique<SuperPowerTongjiPredictor>(
            initial_observation, initial_radius_mm_, config_);
        std::cout << "[ordinary_vehicle_ekf] backend=superpower (Tongji/xiugai)\n";
        return;
    }

    std::shared_ptr<RestFrame> rest;
    {
        std::lock_guard<std::mutex> lock(g_measurement_mutex);
        rest = g_rest_frame.lock();
    }
    if (!rest) {
        throw std::runtime_error(
            "Alliance backend requires AllPredictor runtime RestFrame registration");
    }

    target_number_ = inferTargetNumber(initial_observation);
    ArmorType::ArmorType armor_type = ArmorType::Hero;
    if (target_number_ >= static_cast<int>(ArmorType::Hero) &&
        target_number_ <= static_cast<int>(ArmorType::Base)) {
        armor_type = static_cast<ArmorType::ArmorType>(target_number_);
    }
    alliance_ = std::make_unique<AllianceLightbarPredictor>(
        config_, rest, armor_type);
    alliance_->update(currentMeasurementsForTarget(target_number_),
                      initial_observation.t);
    std::cout << "[ordinary_vehicle_ekf] backend=alliance (NJUST lightbar), target="
              << target_number_ << "\n";
}

void SuperPowerPredictor::update(const EKFTargetObservation& observation)
{
    if (!alliance_backend_) {
        superpower_->update(observation);
        return;
    }
    alliance_->update(currentMeasurementsForTarget(target_number_), observation.t);
}

void SuperPowerPredictor::updatePair(
    const EKFTargetObservation& primary,
    const EKFTargetObservation& secondary)
{
    if (!alliance_backend_) {
        superpower_->updatePair(primary, secondary);
        return;
    }
    (void)secondary;
    alliance_->update(currentMeasurementsForTarget(target_number_), primary.t);
}

void SuperPowerPredictor::missUpdate(double update_time)
{
    if (alliance_backend_) {
        if (alliance_) alliance_->missUpdate(update_time);
    } else if (superpower_) {
        superpower_->missUpdate(update_time);
    }
}

void SuperPowerPredictor::clear()
{
    if (alliance_) alliance_->clear();
    if (superpower_) superpower_->clear();
}

EKFTargetPrediction SuperPowerPredictor::predict(double predict_time) const
{
    if (alliance_backend_) return alliance_ ? alliance_->predict(predict_time) : EKFTargetPrediction{};
    return superpower_ ? superpower_->predict(predict_time) : EKFTargetPrediction{};
}

EKFTargetState SuperPowerPredictor::state() const
{
    if (alliance_backend_) return alliance_ ? alliance_->state() : EKFTargetState{};
    return superpower_ ? superpower_->state() : EKFTargetState{};
}

EKFTargetDebugState SuperPowerPredictor::debugState() const
{
    if (alliance_backend_) return alliance_ ? alliance_->debugState() : EKFTargetDebugState{};
    return superpower_ ? superpower_->debugState() : EKFTargetDebugState{};
}

bool SuperPowerPredictor::ready() const
{
    return alliance_backend_ ? (alliance_ && alliance_->ready())
                             : (superpower_ && superpower_->ready());
}

bool SuperPowerPredictor::hasState() const
{
    return alliance_backend_ ? (alliance_ && alliance_->hasState())
                             : (superpower_ && superpower_->hasState());
}

int SuperPowerPredictor::debugFlipFlag() const
{
    return alliance_backend_ ? (alliance_ ? alliance_->debugFlipFlag() : 1)
                             : (superpower_ ? superpower_->debugFlipFlag() : 1);
}

const char* SuperPowerPredictor::backendName() const
{
    return alliance_backend_ ? "Alliance-NJUST Lightbar EKF"
                             : "SuperPower-Tongji EKF";
}

std::vector<AllianceLightbarPredictor::ProjectedLightbar>
SuperPowerPredictor::projectedLightbars() const
{
    if (alliance_backend_ && alliance_) return alliance_->projectedLightbars();
    return {};
}
