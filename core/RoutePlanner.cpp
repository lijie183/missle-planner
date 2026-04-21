#include "core/RoutePlanner.h"

namespace mission {

RoutePlanner::RoutePlanner() {
    AStarAlgorithm::Options defaults;
    defaults.gridStepDeg = 0.04;
    defaults.altitudeStepMeters = 220.0;
    defaults.safetyClearanceMeters = 320.0;
    defaults.maxIterations = 280000;
    m_algorithm.setOptions(defaults);
}

void RoutePlanner::setThreatZones(const std::vector<ThreatZone>& threats) {
    m_threatZones = threats;
}

void RoutePlanner::setOptions(const AStarAlgorithm::Options& options) {
    m_algorithm.setOptions(options);
}

const AStarAlgorithm::Options& RoutePlanner::options() const {
    return m_algorithm.options();
}

RoutePlanResult RoutePlanner::planRoute(const osgEarth::GeoPoint& start, const osgEarth::GeoPoint& goal) const {
    MissionRequest request;
    request.start = start;
    request.goal = goal;
    request.threats = m_threatZones;

    return m_algorithm.plan(request);
}

}  // namespace mission
