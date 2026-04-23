#pragma once

#include <QDateTime>
#include <QString>

#include <vector>

#include <osgEarth/GeoData>

#include "core/MissionTypes.h"

namespace mission {

struct FlightRouteReport {
    mission::MissileConfig missile;
    mission::TargetConfig target;
    mission::Assignment assignment;
    std::vector<osgEarth::GeoPoint> routeWaypoints;
    std::vector<mission::TelemetrySample> telemetry;
};

struct FlightReportData {
    QString title;
    QDateTime generatedAt;
    std::vector<mission::MissileConfig> missiles;
    std::vector<mission::TargetConfig> targets;
    std::vector<mission::ThreatZone> threats;
    mission::MultiMissionResult planningResult;
    std::vector<FlightRouteReport> routes;
};

QString buildFlightReportHtml(const FlightReportData& report);

}  // namespace mission