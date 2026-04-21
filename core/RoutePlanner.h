#pragma once

#include <vector>

#include <osgEarth/GeoData>

#include "core/AStarAlgorithm.h"
#include "core/MissionTypes.h"

namespace mission {

class RoutePlanner {
public:
    RoutePlanner();

    void setThreatZones(const std::vector<ThreatZone>& threats);
    void setOptions(const AStarAlgorithm::Options& options);
    const AStarAlgorithm::Options& options() const;

    RoutePlanResult planRoute(const osgEarth::GeoPoint& start, const osgEarth::GeoPoint& goal) const;

private:
    std::vector<ThreatZone> m_threatZones;
    AStarAlgorithm m_algorithm;
};

}  // namespace mission
