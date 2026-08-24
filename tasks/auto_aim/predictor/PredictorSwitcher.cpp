#include "predictor/PredictorSwitcher.h"

namespace PredictorType {
    std::vector<std::string> PredictorTypeStrings = {
        "None",
        "RMM",
        "AutoSwitch(should not be used)",
        "SuperPowerEKF"
    };
}

void PredictorSwitcher::clearHistory() {
}


PredictorType::PredictorType PredictorSwitcher::step() {
    return PredictorType::RotationMotionModel; 
}