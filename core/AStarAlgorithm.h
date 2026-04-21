#pragma once

#include "core/MissionTypes.h"

namespace mission {

class AStarAlgorithm {
public:
    struct Options {
        double gridStepDeg = 0.05;
        double altitudeStepMeters = 250.0;
        double safetyClearanceMeters = 300.0;
        double searchMarginDeg = 0.45;
        double threatPenaltyScale = 12000.0;
        int maxIterations = 250000;
        int maxAltitudeLevels = 20;
    };

    AStarAlgorithm();
    explicit AStarAlgorithm(const Options& options);

    void setOptions(const Options& options);
    const Options& options() const;

    RoutePlanResult plan(const MissionRequest& request) const;

private:
    Options m_options;
};

}  // namespace mission
