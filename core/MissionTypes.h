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
        osg::Vec4(248.0f / 255.0f, 194.0f / 255.0f, 79.0f / 255.0f, 1.0f),
        osg::Vec4(119.0f / 255.0f, 234.0f / 255.0f, 145.0f / 255.0f, 1.0f),
        osg::Vec4(86.0f / 255.0f, 214.0f / 255.0f, 255.0f / 255.0f, 1.0f),
        osg::Vec4(236.0f / 255.0f, 121.0f / 255.0f, 193.0f / 255.0f, 1.0f),
        osg::Vec4(166.0f / 255.0f, 145.0f / 255.0f, 245.0f / 255.0f, 1.0f),
        osg::Vec4(255.0f / 255.0f, 155.0f / 255.0f, 105.0f / 255.0f, 1.0f),
        osg::Vec4(255.0f / 255.0f, 107.0f / 255.0f, 107.0f / 255.0f, 1.0f),
        osg::Vec4(78.0f / 255.0f, 205.0f / 255.0f, 196.0f / 255.0f, 1.0f),
        osg::Vec4(255.0f / 255.0f, 230.0f / 255.0f, 109.0f / 255.0f, 1.0f),
        osg::Vec4(168.0f / 255.0f, 230.0f / 255.0f, 207.0f / 255.0f, 1.0f),
    };
    constexpr int n = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
    return palette[static_cast<int>(index) % n];
}

inline osg::Vec4f missileColorF(int index) {
    osg::Vec4 c = missileColor(index);
    return osg::Vec4f(c.r(), c.g(), c.b(), c.a());
}

}  // namespace mission
