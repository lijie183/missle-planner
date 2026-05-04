#pragma once

#include <string>
#include <vector>

#include <osgEarth/GeoData>

namespace mission {

struct ThreatZone {
    double longitudeDeg = 0.0;
    double latitudeDeg = 0.0;
    double radiusMeters = 10000.0;
    double minAltitudeMeters = 0.0;
    double maxAltitudeMeters = 3000.0;
};

struct PlanMetrics {
    bool success = false;
    std::string message;
    double planningTimeMs = 0.0;
    double pathLengthMeters = 0.0;
    int expandedNodes = 0;
    int visitedNodes = 0;
};

struct TelemetrySample {
    double timeSeconds = 0.0;
    double speedMetersPerSecond = 0.0;
    double altitudeMeters = 0.0;
    double pitchDegrees = 0.0;
    double headingDegrees = 0.0;
    double remainingMeters = 0.0;
    double accelerationMetersPerSecond2 = 0.0;
};

struct MissionRequest {
    osgEarth::GeoPoint start;
    osgEarth::GeoPoint goal;
    std::vector<ThreatZone> threats;
    double missileMaxAltitudeMeters = 25000.0;
};

struct RoutePlanResult {
    std::vector<osgEarth::GeoPoint> route;
    PlanMetrics metrics;
};

enum class AllocationMethod {
    Hungarian,
    Genetic,
    Greedy,
    BalancedGreedy
};

enum class RouteAlgorithm {
    AStar,
    Dijkstra,
    RRT,
    HybridAStarPotential
};

struct MissileConfig {
    std::string id;
    std::string name;
    double startLonDeg = 103.80;
    double startLatDeg = 32.10;
    double startAltMeters = 1800.0;
    int missileType = 0;
    double speedMps = 900.0;
    double maxAltitudeMeters = 25000.0;
};

struct TargetConfig {
    std::string id;
    std::string name;
    double lonDeg = 118.40;
    double latDeg = 40.20;
    double altMeters = 2200.0;
    int priority = 5;
};

struct Assignment {
    int missileIndex = -1;
    int targetIndex = -1;
    bool planned = false;
    RoutePlanResult planResult;
    RouteAlgorithm routeAlgorithmUsed = RouteAlgorithm::AStar;
    double estimatedFlightTimeSeconds = 0.0;
    double launchDelaySeconds = 0.0;
    double plannedImpactTimeSeconds = 0.0;
    bool conflictResolved = false;
};

struct RouteConflict {
    int missileA = -1;
    int missileB = -1;
    double minDistanceMeters = 0.0;
    double atTimeSeconds = 0.0;
    bool resolved = false;
    std::string resolution;
};

struct MultiMissionResult {
    std::vector<Assignment> assignments;
    double totalPlanningTimeMs = 0.0;
    int successCount = 0;
    int failureCount = 0;
    double successRate = 0.0;
    int detectedConflictCount = 0;
    int resolvedConflictCount = 0;
    double syncWindowSeconds = 0.0;
    double maxImpactTimeErrorSeconds = 0.0;
    std::vector<RouteConflict> conflicts;
    std::string message;
};

inline const char* routeAlgorithmName(RouteAlgorithm algo) {
    switch (algo) {
        case RouteAlgorithm::AStar:
            return "A*";
        case RouteAlgorithm::Dijkstra:
            return "Dijkstra";
        case RouteAlgorithm::RRT:
            return "RRT";
        case RouteAlgorithm::HybridAStarPotential:
            return "Hybrid(A*+Potential)";
        default:
            return "Unknown";
    }
}

inline osg::Vec4 missileColor(int index) {
    static const osg::Vec4 palette[] = {
        osg::Vec4(1.00f, 0.35f, 0.35f, 1.0f),
        osg::Vec4(0.15f, 0.95f, 0.45f, 1.0f),
        osg::Vec4(0.20f, 0.82f, 1.00f, 1.0f),
        osg::Vec4(1.00f, 0.55f, 0.15f, 1.0f),
        osg::Vec4(1.00f, 0.25f, 0.75f, 1.0f),
        osg::Vec4(0.68f, 0.45f, 1.00f, 1.0f),
        osg::Vec4(1.00f, 0.90f, 0.20f, 1.0f),
        osg::Vec4(0.20f, 1.00f, 0.85f, 1.0f),
        osg::Vec4(0.95f, 0.55f, 1.00f, 1.0f),
        osg::Vec4(0.70f, 1.00f, 0.30f, 1.0f),
    };
    constexpr int n = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
    return palette[static_cast<int>(index) % n];
}

inline osg::Vec4f missileColorF(int index) {
    osg::Vec4 c = missileColor(index);
    return osg::Vec4f(c.r(), c.g(), c.b(), c.a());
}

}  // namespace mission
