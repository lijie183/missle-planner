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

double syntheticTerrainHeight(double lonDeg, double latDeg) {
    const double latWave = std::sin(toRadians(latDeg * 4.0));
    const double lonWave = std::cos(toRadians(lonDeg * 3.0));
    const double detailWave = std::sin(toRadians((lonDeg + latDeg) * 8.0));
    return 260.0 + 180.0 * latWave + 120.0 * lonWave + 60.0 * detailWave;
}

bool pointInThreat(
    double lonDeg,
    double latDeg,
    double altMeters,
    const mission::ThreatZone& threat,
    double marginMeters,
    double missileMaxAltitude) {
    const double h = horizontalDistanceMeters(lonDeg, latDeg, threat.longitudeDeg, threat.latitudeDeg);
    if (h > threat.radiusMeters + marginMeters) {
        return false;
    }
    if (missileMaxAltitude > 0.0 && threat.maxAltitudeMeters >= missileMaxAltitude) {
        return true;
    }
    return altMeters >= (threat.minAltitudeMeters - marginMeters) &&
           altMeters <= (threat.maxAltitudeMeters + marginMeters);
}

bool pointIsSafe(
    double lonDeg,
    double latDeg,
    double altMeters,
    const mission::MissionRequest& request,
    double clearanceMeters) {
    if (altMeters < syntheticTerrainHeight(lonDeg, latDeg) + clearanceMeters) {
        return false;
    }
    for (const auto& threat : request.threats) {
        if (pointInThreat(lonDeg, latDeg, altMeters, threat, 0.0, request.missileMaxAltitudeMeters)) {
            return false;
        }
    }
    return true;
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

    const int totalPasses = std::max(4, m_options.optimizePasses * 2);
    for (int pass = 0; pass < totalPasses; ++pass) {
        std::vector<osgEarth::GeoPoint> next = route;
        for (std::size_t i = 1; i + 1 < route.size(); ++i) {
            const auto& curr = route[i];
            double pushLon = 0.0;
            double pushLat = 0.0;
            double pushAlt = 0.0;

            for (const auto& threat : request.threats) {
                const double h = horizontalDistanceMeters(curr.x(), curr.y(), threat.longitudeDeg, threat.latitudeDeg);
                const double influence = threat.radiusMeters * 2.2;
                if (h > influence) {
                    continue;
                }

                const double dirLon = curr.x() - threat.longitudeDeg;
                const double dirLat = curr.y() - threat.latitudeDeg;
                const double len = std::max(1e-6, std::sqrt(dirLon * dirLon + dirLat * dirLat));
                double w = (influence - h) / influence;
                if (h <= threat.radiusMeters) {
                    w = 1.0 + (threat.radiusMeters - h) / std::max(1.0, threat.radiusMeters) * 3.0;
                }
                pushLon += (dirLon / len) * w;
                pushLat += (dirLat / len) * w;

                bool canFlyOver = request.missileMaxAltitudeMeters <= 0.0 ||
                                  threat.maxAltitudeMeters < request.missileMaxAltitudeMeters;
                if (canFlyOver && curr.z() <= threat.maxAltitudeMeters + 800.0) {
                    double altPushW = (threat.maxAltitudeMeters + 900.0 - curr.z()) * 0.06;
                    if (h <= threat.radiusMeters) {
                        altPushW *= 3.0;
                    }
                    pushAlt += altPushW;
                }
            }

            const double localStep = m_options.threatPushStrength * (1.0 + static_cast<double>(pass) * 0.5);
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

    const double clearance = m_options.astarOptions.safetyClearanceMeters;
    for (int fixPass = 0; fixPass < 6; ++fixPass) {
        bool anyUnsafe = false;
        for (std::size_t i = 1; i + 1 < route.size(); ++i) {
            const auto& p = route[i];
            if (pointIsSafe(p.x(), p.y(), p.z(), request, clearance)) {
                continue;
            }
            anyUnsafe = true;
            double bestAlt = p.z();
            bool fixed = false;
            for (double altUp = p.z() + 500.0; altUp <= 95000.0; altUp += 500.0) {
                if (pointIsSafe(p.x(), p.y(), altUp, request, clearance)) {
                    bestAlt = altUp;
                    fixed = true;
                    break;
                }
            }
            if (fixed) {
                route[i] = osgEarth::GeoPoint(wgs84, p.x(), p.y(), bestAlt, osgEarth::ALTMODE_ABSOLUTE);
            }
        }
        if (!anyUnsafe) {
            break;
        }
    }

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
