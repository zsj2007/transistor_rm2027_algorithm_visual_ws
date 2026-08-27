#pragma once

#ifdef SUPERPOWER_TONGJI_IMPL
#include "EKF/superpower_tongji/SuperPowerPredictor.h"
#else

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "2d_armor_detector/Armor.h"
#include "3d_processing/RestFrame.h"
#include "EKF/superpower_tongji/SuperPowerPredictor.h"
#include "EKF/alliance_njust/AllianceLightbarPredictor.h"

// Compatibility facade used by the existing xiugai business layer.
// Runtime backend is selected by ordinary_vehicle_ekf.backend in the main YAML,
// or configs/ordinary_vehicle_ekf.yaml when the key is not present.
class SuperPowerPredictor {
public:
    SuperPowerPredictor(const EKFTargetObservation& initial_observation,
                        double initial_radius_mm,
                        std::shared_ptr<YAML::Node> config_file_ptr);

    void update(const EKFTargetObservation& observation);
    void updatePair(const EKFTargetObservation& primary,
                    const EKFTargetObservation& secondary);
    void missUpdate(double update_time);
    void clear();

    EKFTargetPrediction predict(double predict_time) const;
    EKFTargetState state() const;
    EKFTargetDebugState debugState() const;
    bool ready() const;
    bool hasState() const;
    int debugFlipFlag() const;

    bool isAllianceBackend() const { return alliance_backend_; }
    const char* backendName() const;
    std::vector<AllianceLightbarPredictor::ProjectedLightbar> projectedLightbars() const;

    // The current project shares one RestFrame instance across ordinary-target
    // predictors. AllPredictor registers it during construction.
    static void setRuntimeRestFrame(const std::shared_ptr<RestFrame>& rest_frame);
    // TargetManager publishes the current detector/PnP frame before AllPredictor
    // consumes it. Alliance uses the raw YOLO lightbar endpoints from this cache.
    static void publishFrameMeasurements(const std::vector<ArmorResult>& measurements);

private:
    static bool readAllianceBackend(const YAML::Node& root);
    static std::vector<ArmorResult> currentMeasurementsForTarget(int target_number);
    int inferTargetNumber(const EKFTargetObservation& initial_observation) const;

    std::shared_ptr<YAML::Node> config_;
    bool alliance_backend_ = false;
    int target_number_ = -1;
    double initial_radius_mm_ = 200.0;
    std::unique_ptr<SuperPowerTongjiPredictor> superpower_;
    std::unique_ptr<AllianceLightbarPredictor> alliance_;
};

#endif
