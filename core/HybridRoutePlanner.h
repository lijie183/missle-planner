#pragma once

#include "core/AStarAlgorithm.h"
#include "core/MissionTypes.h"

namespace mission {

class HybridRoutePlanner {
public:
    struct Options {
        AStarAlgorithm::Options astarOptions;
        int optimizePasses = 4;
        double threatPushStrength = 0.18;
    };

    HybridRoutePlanner();
    explicit HybridRoutePlanner(const Options& options);

    void setOptions(const Options& options);
    const Options& options() const;

    RoutePlanResult plan(const MissionRequest& request) const;

private:
    Options m_options;
};

}  // namespace mission
