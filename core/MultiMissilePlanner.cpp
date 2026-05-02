#include "core/MultiMissilePlanner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include <osgEarth/SpatialReference>

#include "core/HungarianAlgorithm.h"
#include "core/GeneticAllocator.h"
#include "core/RRTAlgorithm.h"
#include "core/HybridRoutePlanner.h"

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

double approxHorizontalDistanceMeters(
    double lonDegA, double latDegA,
    double lonDegB, double latDegB) {
    const double meanLatRad = toRadians((latDegA + latDegB) * 0.5);
    const double metersPerLonDegree = kMetersPerLatDegree * std::max(0.1, std::cos(meanLatRad));
    const double dx = (lonDegB - lonDegA) * metersPerLonDegree;
    const double dy = (latDegB - latDegA) * kMetersPerLatDegree;
    return std::sqrt(dx * dx + dy * dy);
}

double estimateFlightTimeSeconds(const std::vector<osgEarth::GeoPoint>& route, double speedMetersPerSecond) {
    if (route.size() < 2) {
        return 0.0;
    }
    const double safeSpeed = std::max(1.0, speedMetersPerSecond);
    double totalMeters = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i) {
        totalMeters += approxDistanceMeters(
            route[i - 1].x(), route[i - 1].y(), route[i - 1].z(),
            route[i].x(), route[i].y(), route[i].z());
    }
    return totalMeters / safeSpeed;
}

osgEarth::GeoPoint sampleRouteAtTime(
    const std::vector<osgEarth::GeoPoint>& route,
    double flightTimeSeconds,
    double tSeconds) {
    if (route.empty()) {
        return {};
    }
    if (route.size() == 1 || flightTimeSeconds <= 1e-6 || tSeconds <= 0.0) {
        return route.front();
    }

    if (tSeconds >= flightTimeSeconds) {
        return route.back();
    }

    const double ratio = std::clamp(tSeconds / flightTimeSeconds, 0.0, 1.0);
    const double routePos = ratio * static_cast<double>(route.size() - 1);
    const int i0 = static_cast<int>(std::floor(routePos));
    const int i1 = std::min(i0 + 1, static_cast<int>(route.size() - 1));
    const double localT = routePos - static_cast<double>(i0);

    const auto* srs = route.front().getSRS();
    const auto& a = route[static_cast<std::size_t>(i0)];
    const auto& b = route[static_cast<std::size_t>(i1)];
    return osgEarth::GeoPoint(
        srs,
        a.x() + (b.x() - a.x()) * localT,
        a.y() + (b.y() - a.y()) * localT,
        a.z() + (b.z() - a.z()) * localT,
        osgEarth::ALTMODE_ABSOLUTE);
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
        case AllocationMethod::BalancedGreedy:
            return balancedGreedyAllocation(costMatrix, targetPriorities);
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

std::vector<int> MultiMissilePlanner::balancedGreedyAllocation(
    const std::vector<std::vector<double>>& costMatrix,
    const std::vector<int>& targetPriorities) const {
    const int missileCount = static_cast<int>(costMatrix.size());
    if (missileCount == 0) {
        return {};
    }
    const int targetCount = static_cast<int>(costMatrix[0].size());

    std::vector<int> assignment(missileCount, -1);
    std::vector<bool> targetUsed(targetCount, false);
    std::vector<bool> missileUsed(missileCount, false);

    // 优先保障高价值目标，再用距离代价做精细匹配。
    std::vector<int> targetOrder(targetCount);
    for (int t = 0; t < targetCount; ++t) {
        targetOrder[t] = t;
    }

    std::stable_sort(targetOrder.begin(), targetOrder.end(), [&](int a, int b) {
        if (targetPriorities[a] != targetPriorities[b]) {
            return targetPriorities[a] > targetPriorities[b];
        }

        double bestA = std::numeric_limits<double>::infinity();
        double bestB = std::numeric_limits<double>::infinity();
        for (int m = 0; m < missileCount; ++m) {
            bestA = std::min(bestA, costMatrix[m][a]);
            bestB = std::min(bestB, costMatrix[m][b]);
        }
        return bestA < bestB;
    });

    for (int targetIdx : targetOrder) {
        if (targetUsed[targetIdx]) {
            continue;
        }

        int bestMissile = -1;
        double bestScore = std::numeric_limits<double>::infinity();
        for (int missileIdx = 0; missileIdx < missileCount; ++missileIdx) {
            if (missileUsed[missileIdx]) {
                continue;
            }

            const double distanceCost = costMatrix[missileIdx][targetIdx];
            const double priorityGain = std::max(1, targetPriorities[targetIdx]);
            const double score = distanceCost / static_cast<double>(priorityGain);
            if (score < bestScore) {
                bestScore = score;
                bestMissile = missileIdx;
            }
        }

        if (bestMissile >= 0) {
            assignment[bestMissile] = targetIdx;
            missileUsed[bestMissile] = true;
            targetUsed[targetIdx] = true;
        }
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
    AStarAlgorithm::Options dijkstraOptions = options.astarOptions;
    dijkstraOptions.heuristicWeight = 0.0;
    AStarAlgorithm dijkstra(dijkstraOptions);

    RRTAlgorithm::Options rrtOptions;
    rrtOptions.safetyClearanceMeters = options.astarOptions.safetyClearanceMeters;
    rrtOptions.searchMarginDeg = options.astarOptions.searchMarginDeg;
    rrtOptions.maxIterations = std::max(800, options.astarOptions.maxIterations / 25);
    RRTAlgorithm rrt(rrtOptions);

    HybridRoutePlanner::Options hybridOptions;
    hybridOptions.astarOptions = options.astarOptions;
    HybridRoutePlanner hybrid(hybridOptions);

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

                switch (options.routeAlgorithm) {
                    case RouteAlgorithm::AStar:
                        assign.planResult = astar.plan(request);
                        break;
                    case RouteAlgorithm::Dijkstra:
                        assign.planResult = dijkstra.plan(request);
                        break;
                    case RouteAlgorithm::RRT:
                        assign.planResult = rrt.plan(request);
                        break;
                    case RouteAlgorithm::HybridAStarPotential:
                        assign.planResult = hybrid.plan(request);
                        break;
                    default:
                        assign.planResult = astar.plan(request);
                        break;
                }

                assign.planned = assign.planResult.metrics.success;
                assign.routeAlgorithmUsed = options.routeAlgorithm;
                if (assign.planned) {
                    assign.estimatedFlightTimeSeconds = estimateFlightTimeSeconds(
                        assign.planResult.route,
                        missiles[static_cast<std::size_t>(i)].speedMps);
                    assign.plannedImpactTimeSeconds = assign.estimatedFlightTimeSeconds;
                }
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

    detectAndResolveConflicts(result, missiles, threats, options);
    applyTimeSynchronization(result, missiles, options);

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

void MultiMissilePlanner::detectAndResolveConflicts(
    MultiMissionResult& result,
    const std::vector<MissileConfig>& missiles,
    const std::vector<ThreatZone>& threats,
    const Options& options) const {
    result.conflicts.clear();
    result.detectedConflictCount = 0;
    result.resolvedConflictCount = 0;

    const double threshold = std::max(800.0, options.conflictDistanceMeters);

    for (int i = 0; i < static_cast<int>(result.assignments.size()); ++i) {
        auto& a = result.assignments[static_cast<std::size_t>(i)];
        if (!a.planned || a.planResult.route.size() < 2) {
            continue;
        }

        for (int j = i + 1; j < static_cast<int>(result.assignments.size()); ++j) {
            auto& b = result.assignments[static_cast<std::size_t>(j)];
            if (!b.planned || b.planResult.route.size() < 2) {
                continue;
            }

            const double ta = std::max(1.0, a.estimatedFlightTimeSeconds);
            const double tb = std::max(1.0, b.estimatedFlightTimeSeconds);
            const double horizon = std::max(ta, tb);

            double minDist = std::numeric_limits<double>::infinity();
            double atTime = 0.0;

            constexpr int kSamples = 48;
            for (int s = 0; s <= kSamples; ++s) {
                const double t = horizon * static_cast<double>(s) / static_cast<double>(kSamples);
                const osgEarth::GeoPoint pa = sampleRouteAtTime(a.planResult.route, ta, t);
                const osgEarth::GeoPoint pb = sampleRouteAtTime(b.planResult.route, tb, t);
                const double d = approxDistanceMeters(pa.x(), pa.y(), pa.z(), pb.x(), pb.y(), pb.z());
                if (d < minDist) {
                    minDist = d;
                    atTime = t;
                }
            }

            if (minDist >= threshold) {
                continue;
            }

            RouteConflict conflict;
            conflict.missileA = a.missileIndex;
            conflict.missileB = b.missileIndex;
            conflict.minDistanceMeters = minDist;
            conflict.atTimeSeconds = atTime;
            conflict.resolved = false;
            conflict.resolution = "detected";

            if (options.enableConflictResolution) {
                const double offsetMeters = threshold - minDist + 900.0;
                for (std::size_t k = 1; k + 1 < b.planResult.route.size(); ++k) {
                    auto p = b.planResult.route[k];
                    double newAlt = p.z() + offsetMeters;
                    bool inAnyThreat = false;
                    for (const auto& threat : threats) {
                        const double h = approxHorizontalDistanceMeters(p.x(), p.y(), threat.longitudeDeg, threat.latitudeDeg);
                        if (h <= threat.radiusMeters &&
                            newAlt >= threat.minAltitudeMeters &&
                            newAlt <= threat.maxAltitudeMeters) {
                            inAnyThreat = true;
                            break;
                        }
                    }
                    if (inAnyThreat) {
                        newAlt = 0.0;
                        for (double altTry = p.z() + offsetMeters; altTry <= 95000.0; altTry += 1000.0) {
                            bool safe = true;
                            for (const auto& threat : threats) {
                                const double h2 = approxHorizontalDistanceMeters(p.x(), p.y(), threat.longitudeDeg, threat.latitudeDeg);
                                if (h2 <= threat.radiusMeters &&
                                    altTry >= threat.minAltitudeMeters &&
                                    altTry <= threat.maxAltitudeMeters) {
                                    safe = false;
                                    break;
                                }
                            }
                            if (safe) {
                                newAlt = altTry;
                                break;
                            }
                        }
                        if (newAlt <= 0.0) {
                            newAlt = p.z() + offsetMeters;
                        }
                    }
                    p.z() = newAlt;
                    b.planResult.route[k] = p;
                }
                b.conflictResolved = true;
                conflict.resolved = true;
                conflict.resolution = "altitude-offset";
                ++result.resolvedConflictCount;
            }

            result.conflicts.push_back(conflict);
            ++result.detectedConflictCount;
        }
    }
}

void MultiMissilePlanner::applyTimeSynchronization(
    MultiMissionResult& result,
    const std::vector<MissileConfig>& missiles,
    const Options& options) const {
    result.syncWindowSeconds = 0.0;
    result.maxImpactTimeErrorSeconds = 0.0;

    if (!options.enableTimeSynchronization) {
        return;
    }

    double latestImpact = 0.0;
    for (const auto& a : result.assignments) {
        if (!a.planned) {
            continue;
        }
        latestImpact = std::max(latestImpact, a.plannedImpactTimeSeconds);
    }

    if (latestImpact <= 1e-6) {
        return;
    }

    double earliestImpact = latestImpact;
    double maxError = 0.0;

    for (auto& a : result.assignments) {
        if (!a.planned) {
            continue;
        }

        const double delay = std::max(0.0, latestImpact - a.estimatedFlightTimeSeconds);
        a.launchDelaySeconds = delay;
        a.plannedImpactTimeSeconds = a.estimatedFlightTimeSeconds + a.launchDelaySeconds;
        earliestImpact = std::min(earliestImpact, a.plannedImpactTimeSeconds);
        maxError = std::max(maxError, std::abs(a.plannedImpactTimeSeconds - latestImpact));
    }

    result.syncWindowSeconds = std::max(0.0, latestImpact - earliestImpact);
    result.maxImpactTimeErrorSeconds = maxError;

    if (result.syncWindowSeconds > options.targetSyncWindowSeconds) {
        for (auto& a : result.assignments) {
            if (!a.planned) {
                continue;
            }

            const double compress = options.targetSyncWindowSeconds / std::max(1e-3, result.syncWindowSeconds);
            a.launchDelaySeconds *= std::clamp(compress, 0.35, 1.0);
            a.plannedImpactTimeSeconds = a.estimatedFlightTimeSeconds + a.launchDelaySeconds;
        }

        double latest = 0.0;
        double earliest = std::numeric_limits<double>::infinity();
        for (const auto& a : result.assignments) {
            if (!a.planned) {
                continue;
            }
            latest = std::max(latest, a.plannedImpactTimeSeconds);
            earliest = std::min(earliest, a.plannedImpactTimeSeconds);
        }

        result.syncWindowSeconds = std::max(0.0, latest - earliest);
        result.maxImpactTimeErrorSeconds = result.syncWindowSeconds;
    }
}

}  // namespace mission
