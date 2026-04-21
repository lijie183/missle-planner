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

struct MissionRequest {
    osgEarth::GeoPoint start;
    osgEarth::GeoPoint goal;
    std::vector<ThreatZone> threats;
};

struct RoutePlanResult {
    std::vector<osgEarth::GeoPoint> route;
    PlanMetrics metrics;
};

}  // namespace mission
