#pragma once

#include "core/MissionTypes.h"

namespace mission {

class RRTAlgorithm {
public:
    struct Options {
        double stepMeters = 18000.0;
        double connectDistanceMeters = 22000.0;
        double goalBias = 0.20;
        double safetyClearanceMeters = 300.0;
        double searchMarginDeg = 0.45;
        int maxIterations = 5000;
        int randomSeed = 42;
    };

    RRTAlgorithm();
    explicit RRTAlgorithm(const Options& options);

    void setOptions(const Options& options);
    const Options& options() const;

    RoutePlanResult plan(const MissionRequest& request) const;

private:
    Options m_options;
};

}  // namespace mission
