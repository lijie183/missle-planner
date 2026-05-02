#include "core/HybridRoutePlanner.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <osgEarth/SpatialReference>

namespace {

constexpr double kMetersPerLatDegree = 111320.0;

double toRadians(double degrees) {
    return degrees * 0.017453292519943295;
}

double metersPerLonDegree(double latDeg) {
    return kMetersPerLatDegree * std::max(0.1, std::cos(toRadians(latDeg)));
}

double horizontalDistanceMeters(double lonA, double latA, double lonB, double latB) {
    const double meanLat = (latA + latB) * 0.5;
    const double dx = (lonB - lonA) * metersPerLonDegree(meanLat);
    const double dy = (latB - latA) * kMetersPerLatDegree;
    return std::sqrt(dx * dx + dy * dy);
}

double routeLengthMeters(const std::vector<osgEarth::GeoPoint>& route) {
    if (route.size() < 2) {
        return 0.0;
    }
    double sum = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i) {
        const double h = horizontalDistanceMeters(route[i - 1].x(), route[i - 1].y(), route[i].x(), route[i].y());
        const double dz = route[i].z() - route[i - 1].z();
        sum += std::sqrt(h * h + dz * dz);
    }
    return sum;
}

}  // namespace

namespace mission {

HybridRoutePlanner::HybridRoutePlanner() = default;

HybridRoutePlanner::HybridRoutePlanner(const Options& options)
    : m_options(options) {}

void HybridRoutePlanner::setOptions(const Options& options) {
    m_options = options;
}

const HybridRoutePlanner::Options& HybridRoutePlanner::options() const {
    return m_options;
}

RoutePlanResult HybridRoutePlanner::plan(const MissionRequest& request) const {
    AStarAlgorithm astar(m_options.astarOptions);
    RoutePlanResult base = astar.plan(request);
    if (!base.metrics.success || base.route.size() < 3) {
        return base;
    }

    std::vector<osgEarth::GeoPoint> route = base.route;
    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (wgs84 == nullptr) {
        return base;
    }

    for (int pass = 0; pass < std::max(1, m_options.optimizePasses); ++pass) {
        std::vector<osgEarth::GeoPoint> next = route;
        for (std::size_t i = 1; i + 1 < route.size(); ++i) {
            const auto& curr = route[i];
            double pushLon = 0.0;
            double pushLat = 0.0;
            double pushAlt = 0.0;

            for (const auto& threat : request.threats) {
                const double h = horizontalDistanceMeters(curr.x(), curr.y(), threat.longitudeDeg, threat.latitudeDeg);
                const double influence = threat.radiusMeters * 1.7;
                if (h > influence) {
                    continue;
                }

                const double dirLon = curr.x() - threat.longitudeDeg;
                const double dirLat = curr.y() - threat.latitudeDeg;
                const double len = std::max(1e-6, std::sqrt(dirLon * dirLon + dirLat * dirLat));
                const double w = (influence - h) / influence;
                pushLon += (dirLon / len) * w;
                pushLat += (dirLat / len) * w;

                if (curr.z() <= threat.maxAltitudeMeters + 800.0) {
                    pushAlt += (threat.maxAltitudeMeters + 900.0 - curr.z()) * 0.06;
                }
            }

            const double localStep = m_options.threatPushStrength * (1.0 + static_cast<double>(pass) * 0.35);
            const double lonDelta = pushLon * localStep * 0.010;
            const double latDelta = pushLat * localStep * 0.010;
            const double altDelta = pushAlt * localStep;

            const double smoothLon = (route[i - 1].x() + route[i + 1].x()) * 0.5;
            const double smoothLat = (route[i - 1].y() + route[i + 1].y()) * 0.5;
            const double smoothAlt = (route[i - 1].z() + route[i + 1].z()) * 0.5;

            const double lon = curr.x() * 0.58 + smoothLon * 0.42 + lonDelta;
            const double lat = curr.y() * 0.58 + smoothLat * 0.42 + latDelta;
            const double alt = std::max(0.0, curr.z() * 0.60 + smoothAlt * 0.40 + altDelta);

            next[i] = osgEarth::GeoPoint(wgs84, lon, lat, alt, osgEarth::ALTMODE_ABSOLUTE);
        }
        route.swap(next);
    }

    route.front() = base.route.front();
    route.back() = base.route.back();

    const double oldLength = base.metrics.pathLengthMeters;
    const double newLength = routeLengthMeters(route);

    base.route = route;
    base.metrics.pathLengthMeters = newLength;
    base.metrics.message = "Hybrid(A*+Potential)规划成功。";

    if (oldLength > 1e-6 && newLength < oldLength) {
        const double improveRate = (oldLength - newLength) / oldLength * 100.0;
        base.metrics.message += " 路径长度改进 " + std::to_string(improveRate).substr(0, 5) + "%";
    }

    return base;
}

}  // namespace mission
