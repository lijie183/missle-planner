#pragma once

#include <vector>

#include <osgEarth/GeoData>

#include "core/AStarAlgorithm.h"
#include "core/MissionTypes.h"

namespace mission {

class MultiMissilePlanner {
public:
    struct Options {
        AllocationMethod method = AllocationMethod::Hungarian;
        AStarAlgorithm::Options astarOptions;
    };

    MultiMissionResult plan(
        const std::vector<MissileConfig>& missiles,
        const std::vector<TargetConfig>& targets,
        const std::vector<ThreatZone>& threats,
        const Options& options) const;

    MultiMissionResult replan(
        const std::vector<MissileConfig>& missiles,
        const std::vector<TargetConfig>& targets,
        const std::vector<ThreatZone>& threats,
        const Options& options,
        const std::vector<int>& failedMissileIndices,
        const std::vector<int>& completedTargetIndices) const;

private:
    std::vector<std::vector<double>> buildCostMatrix(
        const std::vector<MissileConfig>& missiles,
        const std::vector<TargetConfig>& targets) const;

    std::vector<int> runAllocation(
        const std::vector<std::vector<double>>& costMatrix,
        const std::vector<int>& targetPriorities,
        AllocationMethod method) const;

    std::vector<int> greedyAllocation(
        const std::vector<std::vector<double>>& costMatrix,
        const std::vector<int>& targetPriorities) const;

    std::vector<int> balancedGreedyAllocation(
        const std::vector<std::vector<double>>& costMatrix,
        const std::vector<int>& targetPriorities) const;
};

}  // namespace mission
