#include "core/MultiMissilePlanner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <osgEarth/SpatialReference>

#include "core/HungarianAlgorithm.h"
#include "core/GeneticAllocator.h"

namespace {

constexpr double kMetersPerLatDegree = 111320.0;

double toRadians(double degrees) {
    return degrees * 0.017453292519943295;
}

double approxDistanceMeters(
    double lonDegA, double latDegA, double altA,
    double lonDegB, double latDegB, double altB) {
    const double meanLatRad = toRadians((latDegA + latDegB) * 0.5);
    const double metersPerLonDegree = kMetersPerLatDegree * std::max(0.1, std::cos(meanLatRad));
    const double dx = (lonDegB - lonDegA) * metersPerLonDegree;
    const double dy = (latDegB - latDegA) * kMetersPerLatDegree;
    const double dz = altB - altA;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

namespace mission {

std::vector<std::vector<double>> MultiMissilePlanner::buildCostMatrix(
    const std::vector<MissileConfig>& missiles,
    const std::vector<TargetConfig>& targets) const {
    const int n = static_cast<int>(missiles.size());
    const int m = static_cast<int>(targets.size());
    std::vector<std::vector<double>> cost(n, std::vector<double>(m, 0.0));

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            const double dist = approxDistanceMeters(
                missiles[i].startLonDeg, missiles[i].startLatDeg, missiles[i].startAltMeters,
                targets[j].lonDeg, targets[j].latDeg, targets[j].altMeters);

            const double priorityFactor = (11.0 - targets[j].priority) / 10.0;
            cost[i][j] = dist * priorityFactor;
        }
    }

    return cost;
}

std::vector<int> MultiMissilePlanner::runAllocation(
    const std::vector<std::vector<double>>& costMatrix,
    const std::vector<int>& targetPriorities,
    AllocationMethod method) const {
    switch (method) {
        case AllocationMethod::Hungarian:
            return HungarianAlgorithm::solve(costMatrix);
        case AllocationMethod::Genetic:
            return GeneticAllocator::solve(costMatrix, targetPriorities);
        case AllocationMethod::Greedy:
            return greedyAllocation(costMatrix, targetPriorities);
        default:
            return greedyAllocation(costMatrix, targetPriorities);
    }
}

std::vector<int> MultiMissilePlanner::greedyAllocation(
    const std::vector<std::vector<double>>& costMatrix,
    const std::vector<int>& targetPriorities) const {
    const int n = static_cast<int>(costMatrix.size());
    if (n == 0) {
        return {};
    }
    const int m = static_cast<int>(costMatrix[0].size());

    std::vector<int> assignment(n, -1);
    std::vector<bool> targetUsed(m, false);

    std::vector<std::tuple<double, int, int>> candidates;
    candidates.reserve(static_cast<std::size_t>(n * m));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            double adjustedCost = costMatrix[i][j] / std::max(1, targetPriorities[j]);
            candidates.emplace_back(adjustedCost, i, j);
        }
    }
    std::sort(candidates.begin(), candidates.end());

    for (const auto& [cost, missileIdx, targetIdx] : candidates) {
        if (assignment[missileIdx] >= 0 || targetUsed[targetIdx]) {
            continue;
        }
        assignment[missileIdx] = targetIdx;
        targetUsed[targetIdx] = true;
    }

    return assignment;
}

MultiMissionResult MultiMissilePlanner::plan(
    const std::vector<MissileConfig>& missiles,
    const std::vector<TargetConfig>& targets,
    const std::vector<ThreatZone>& threats,
    const Options& options) const {
    MultiMissionResult result;
    const auto startTime = std::chrono::steady_clock::now();

    const int nMissiles = static_cast<int>(missiles.size());
    const int nTargets = static_cast<int>(targets.size());

    if (nMissiles == 0 || nTargets == 0) {
        result.message = "导弹或目标列表为空。";
        result.totalPlanningTimeMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();
        return result;
    }

    const auto costMatrix = buildCostMatrix(missiles, targets);
    std::vector<int> targetPriorities(nTargets);
    for (int j = 0; j < nTargets; ++j) {
        targetPriorities[j] = targets[j].priority;
    }

    const auto allocation = runAllocation(costMatrix, targetPriorities, options.method);

    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");
    AStarAlgorithm astar(options.astarOptions);

    result.assignments.reserve(static_cast<std::size_t>(nMissiles));
    for (int i = 0; i < nMissiles; ++i) {
        Assignment assign;
        assign.missileIndex = i;

        if (i < static_cast<int>(allocation.size()) && allocation[i] >= 0 && allocation[i] < nTargets) {
            assign.targetIndex = allocation[i];

            if (wgs84 != nullptr) {
                MissionRequest request;
                request.start = osgEarth::GeoPoint(
                    wgs84, missiles[i].startLonDeg, missiles[i].startLatDeg,
                    missiles[i].startAltMeters, osgEarth::ALTMODE_ABSOLUTE);
                request.goal = osgEarth::GeoPoint(
                    wgs84, targets[assign.targetIndex].lonDeg, targets[assign.targetIndex].latDeg,
                    targets[assign.targetIndex].altMeters, osgEarth::ALTMODE_ABSOLUTE);
                request.threats = threats;

                assign.planResult = astar.plan(request);
                assign.planned = assign.planResult.metrics.success;
            }
        }

        result.assignments.push_back(assign);
    }

    result.successCount = 0;
    result.failureCount = 0;
    for (const auto& a : result.assignments) {
        if (a.planned) {
            ++result.successCount;
        } else {
            ++result.failureCount;
        }
    }

    const int totalTargets = nTargets;
    const int hitTargets = std::min(result.successCount, totalTargets);
    result.successRate = totalTargets > 0 ? static_cast<double>(hitTargets) / totalTargets : 0.0;

    result.message = "多导弹规划完成。";
    result.totalPlanningTimeMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();

    return result;
}

MultiMissionResult MultiMissilePlanner::replan(
    const std::vector<MissileConfig>& missiles,
    const std::vector<TargetConfig>& targets,
    const std::vector<ThreatZone>& threats,
    const Options& options,
    const std::vector<int>& failedMissileIndices,
    const std::vector<int>& completedTargetIndices) const {
    std::vector<MissileConfig> activeMissiles;
    std::vector<int> missileRemap;
    for (int i = 0; i < static_cast<int>(missiles.size()); ++i) {
        bool failed = false;
        for (int fi : failedMissileIndices) {
            if (fi == i) {
                failed = true;
                break;
            }
        }
        if (!failed) {
            missileRemap.push_back(i);
            activeMissiles.push_back(missiles[i]);
        }
    }

    std::vector<TargetConfig> remainingTargets;
    std::vector<int> targetRemap;
    for (int j = 0; j < static_cast<int>(targets.size()); ++j) {
        bool completed = false;
        for (int ci : completedTargetIndices) {
            if (ci == j) {
                completed = true;
                break;
            }
        }
        if (!completed) {
            targetRemap.push_back(j);
            remainingTargets.push_back(targets[j]);
        }
    }

    MultiMissionResult partial = plan(activeMissiles, remainingTargets, threats, options);

    MultiMissionResult result;
    result.totalPlanningTimeMs = partial.totalPlanningTimeMs;
    result.message = "动态重规划完成。";

    for (const auto& a : partial.assignments) {
        Assignment remapped;
        remapped.missileIndex = missileRemap[a.missileIndex];
        remapped.targetIndex = a.targetIndex >= 0 ? targetRemap[a.targetIndex] : -1;
        remapped.planned = a.planned;
        remapped.planResult = a.planResult;
        result.assignments.push_back(remapped);
    }

    result.successCount = partial.successCount;
    result.failureCount = partial.failureCount;
    const int totalTargets = static_cast<int>(targets.size()) - static_cast<int>(completedTargetIndices.size());
    result.successRate = totalTargets > 0
        ? static_cast<double>(partial.successCount) / totalTargets
        : 0.0;

    return result;
}

}  // namespace mission
