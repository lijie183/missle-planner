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
};

struct RoutePlanResult {
    std::vector<osgEarth::GeoPoint> route;
    PlanMetrics metrics;
};

enum class AllocationMethod {
    Hungarian,
    Genetic,
    Greedy
};

struct MissileConfig {
    std::string id;
    std::string name;
    double startLonDeg = 112.35;
    double startLatDeg = 34.70;
    double startAltMeters = 1200.0;
    int missileType = 0;
    double speedMps = 250.0;
};

struct TargetConfig {
    std::string id;
    std::string name;
    double lonDeg = 113.30;
    double latDeg = 35.15;
    double altMeters = 1600.0;
    int priority = 5;
};

struct Assignment {
    int missileIndex = -1;
    int targetIndex = -1;
    bool planned = false;
    RoutePlanResult planResult;
};

struct MultiMissionResult {
    std::vector<Assignment> assignments;
    double totalPlanningTimeMs = 0.0;
    int successCount = 0;
    int failureCount = 0;
    double successRate = 0.0;
    std::string message;
};

inline osg::Vec4 missileColor(int index) {
    static const osg::Vec4 palette[] = {
        osg::Vec4(0.00f, 0.90f, 1.00f, 1.00f),
        osg::Vec4(1.00f, 0.20f, 0.80f, 1.00f),
        osg::Vec4(1.00f, 0.84f, 0.00f, 1.00f),
        osg::Vec4(0.46f, 1.00f, 0.01f, 1.00f),
        osg::Vec4(1.00f, 0.43f, 0.00f, 1.00f),
        osg::Vec4(1.00f, 0.09f, 0.26f, 1.00f),
        osg::Vec4(0.67f, 0.00f, 1.00f, 1.00f),
        osg::Vec4(0.78f, 1.00f, 0.00f, 1.00f),
        osg::Vec4(0.00f, 0.76f, 0.62f, 1.00f),
        osg::Vec4(0.96f, 0.56f, 0.26f, 1.00f),
        osg::Vec4(0.55f, 0.76f, 1.00f, 1.00f),
        osg::Vec4(0.93f, 0.35f, 0.35f, 1.00f),
    };
    constexpr int n = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
    return palette[static_cast<int>(index) % n];
}

inline osg::Vec4f missileColorF(int index) {
    osg::Vec4 c = missileColor(index);
    return osg::Vec4f(c.r(), c.g(), c.b(), c.a());
}

}  // namespace mission
