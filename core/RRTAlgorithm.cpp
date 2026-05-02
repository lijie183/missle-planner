#include "core/RRTAlgorithm.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <osgEarth/SpatialReference>

namespace {

constexpr double kMetersPerLatDegree = 111320.0;

struct Node {
    double lonDeg = 0.0;
    double latDeg = 0.0;
    double altMeters = 0.0;
    int parentIndex = -1;
};

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

double distance3dMeters(
    double lonA,
    double latA,
    double altA,
    double lonB,
    double latB,
    double altB) {
    const double h = horizontalDistanceMeters(lonA, latA, lonB, latB);
    const double dz = altB - altA;
    return std::sqrt(h * h + dz * dz);
}

double syntheticTerrainHeight(double lonDeg, double latDeg) {
    const double latWave = std::sin(toRadians(latDeg * 4.0));
    const double lonWave = std::cos(toRadians(lonDeg * 3.0));
    const double detailWave = std::sin(toRadians((lonDeg + latDeg) * 8.0));
    return 260.0 + 180.0 * latWave + 120.0 * lonWave + 60.0 * detailWave;
}

bool inThreat(
    double lonDeg,
    double latDeg,
    double altMeters,
    const mission::ThreatZone& threat) {
    const double h = horizontalDistanceMeters(lonDeg, latDeg, threat.longitudeDeg, threat.latitudeDeg);
    if (h > threat.radiusMeters) {
        return false;
    }
    return altMeters >= threat.minAltitudeMeters && altMeters <= threat.maxAltitudeMeters;
}

bool pointSafe(
    double lonDeg,
    double latDeg,
    double altMeters,
    const mission::MissionRequest& request,
    const mission::RRTAlgorithm::Options& options) {
    if (altMeters < syntheticTerrainHeight(lonDeg, latDeg) + options.safetyClearanceMeters) {
        return false;
    }
    for (const auto& threat : request.threats) {
        if (inThreat(lonDeg, latDeg, altMeters, threat)) {
            return false;
        }
    }
    return true;
}

bool segmentSafe(
    const Node& a,
    const Node& b,
    const mission::MissionRequest& request,
    const mission::RRTAlgorithm::Options& options) {
    constexpr int kSamples = 16;
    for (int i = 0; i <= kSamples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(kSamples);
        const double lon = a.lonDeg + (b.lonDeg - a.lonDeg) * t;
        const double lat = a.latDeg + (b.latDeg - a.latDeg) * t;
        const double alt = a.altMeters + (b.altMeters - a.altMeters) * t;
        if (!pointSafe(lon, lat, alt, request, options)) {
            return false;
        }
    }
    return true;
}

std::vector<osgEarth::GeoPoint> shortcutSmooth(
    std::vector<osgEarth::GeoPoint> route,
    const mission::MissionRequest& request,
    const mission::RRTAlgorithm::Options& options) {
    if (route.size() < 4) {
        return route;
    }

    std::mt19937 rng(1337);
    std::uniform_int_distribution<int> pick(1, static_cast<int>(route.size()) - 2);

    const auto* wgs84 = route.front().getSRS();
    for (int iter = 0; iter < 120; ++iter) {
        int i = pick(rng);
        int j = pick(rng);
        if (std::abs(i - j) < 2) {
            continue;
        }
        if (i > j) {
            std::swap(i, j);
        }

        Node a{route[static_cast<std::size_t>(i - 1)].x(), route[static_cast<std::size_t>(i - 1)].y(), route[static_cast<std::size_t>(i - 1)].z(), -1};
        Node b{route[static_cast<std::size_t>(j + 1)].x(), route[static_cast<std::size_t>(j + 1)].y(), route[static_cast<std::size_t>(j + 1)].z(), -1};
        if (!segmentSafe(a, b, request, options)) {
            continue;
        }

        std::vector<osgEarth::GeoPoint> next;
        next.reserve(route.size() - static_cast<std::size_t>(j - i));
        for (int k = 0; k < i; ++k) {
            next.push_back(route[static_cast<std::size_t>(k)]);
        }
        next.push_back(osgEarth::GeoPoint(wgs84, b.lonDeg, b.latDeg, b.altMeters, osgEarth::ALTMODE_ABSOLUTE));
        for (std::size_t k = static_cast<std::size_t>(j + 1); k < route.size(); ++k) {
            next.push_back(route[k]);
        }
        route.swap(next);
        if (route.size() < 4) {
            break;
        }
        pick = std::uniform_int_distribution<int>(1, static_cast<int>(route.size()) - 2);
    }

    return route;
}

double routeLengthMeters(const std::vector<osgEarth::GeoPoint>& route) {
    if (route.size() < 2) {
        return 0.0;
    }
    double sum = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i) {
        sum += distance3dMeters(
            route[i - 1].x(), route[i - 1].y(), route[i - 1].z(),
            route[i].x(), route[i].y(), route[i].z());
    }
    return sum;
}

}  // namespace

namespace mission {

RRTAlgorithm::RRTAlgorithm() = default;

RRTAlgorithm::RRTAlgorithm(const Options& options)
    : m_options(options) {}

void RRTAlgorithm::setOptions(const Options& options) {
    m_options = options;
}

const RRTAlgorithm::Options& RRTAlgorithm::options() const {
    return m_options;
}

RoutePlanResult RRTAlgorithm::plan(const MissionRequest& request) const {
    RoutePlanResult result;
    const auto begin = std::chrono::steady_clock::now();

    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (wgs84 == nullptr) {
        result.metrics.message = "无法获取WGS84坐标参考。";
        return result;
    }

    const double minLonBase = std::min(request.start.x(), request.goal.x());
    const double maxLonBase = std::max(request.start.x(), request.goal.x());
    const double minLatBase = std::min(request.start.y(), request.goal.y());
    const double maxLatBase = std::max(request.start.y(), request.goal.y());

    double minLon = minLonBase - m_options.searchMarginDeg;
    double maxLon = maxLonBase + m_options.searchMarginDeg;
    double minLat = minLatBase - m_options.searchMarginDeg;
    double maxLat = maxLatBase + m_options.searchMarginDeg;

    for (const auto& threat : request.threats) {
        const double lonSpan = threat.radiusMeters / metersPerLonDegree(threat.latitudeDeg);
        const double latSpan = threat.radiusMeters / kMetersPerLatDegree;
        minLon = std::min(minLon, threat.longitudeDeg - lonSpan - 0.08);
        maxLon = std::max(maxLon, threat.longitudeDeg + lonSpan + 0.08);
        minLat = std::min(minLat, threat.latitudeDeg - latSpan - 0.08);
        maxLat = std::max(maxLat, threat.latitudeDeg + latSpan + 0.08);
    }

    const double startAlt = std::max(request.start.z(), syntheticTerrainHeight(request.start.x(), request.start.y()) + m_options.safetyClearanceMeters);
    const double goalAlt = std::max(request.goal.z(), syntheticTerrainHeight(request.goal.x(), request.goal.y()) + m_options.safetyClearanceMeters);

    Node start{request.start.x(), request.start.y(), startAlt, -1};
    Node goal{request.goal.x(), request.goal.y(), goalAlt, -1};

    if (!pointSafe(start.lonDeg, start.latDeg, start.altMeters, request, m_options)) {
        result.metrics.message = "RRT起点不可达。";
        return result;
    }
    if (!pointSafe(goal.lonDeg, goal.latDeg, goal.altMeters, request, m_options)) {
        result.metrics.message = "RRT终点不可达。";
        return result;
    }

    std::mt19937 rng(static_cast<std::mt19937::result_type>(m_options.randomSeed));
    std::uniform_real_distribution<double> lonDist(minLon, maxLon);
    std::uniform_real_distribution<double> latDist(minLat, maxLat);
    std::uniform_real_distribution<double> altDist(
        std::min(startAlt, goalAlt),
        std::max(startAlt, goalAlt) + 20000.0);
    std::uniform_real_distribution<double> pick01(0.0, 1.0);

    std::vector<Node> nodes;
    nodes.reserve(static_cast<std::size_t>(m_options.maxIterations + 1));
    nodes.push_back(start);

    int reachedIndex = -1;

    for (int iter = 0; iter < m_options.maxIterations; ++iter) {
        Node sample;
        if (pick01(rng) < std::clamp(m_options.goalBias, 0.0, 0.8)) {
            sample = goal;
        } else {
            sample.lonDeg = lonDist(rng);
            sample.latDeg = latDist(rng);
            sample.altMeters = altDist(rng);
        }

        int nearest = 0;
        double nearestDist = std::numeric_limits<double>::infinity();
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
            const auto& n = nodes[static_cast<std::size_t>(i)];
            const double d = distance3dMeters(n.lonDeg, n.latDeg, n.altMeters, sample.lonDeg, sample.latDeg, sample.altMeters);
            if (d < nearestDist) {
                nearestDist = d;
                nearest = i;
            }
        }

        if (!std::isfinite(nearestDist) || nearestDist < 1.0) {
            continue;
        }

        const Node& from = nodes[static_cast<std::size_t>(nearest)];
        const double step = std::min(std::max(800.0, m_options.stepMeters), nearestDist);
        const double t = std::clamp(step / nearestDist, 0.0, 1.0);

        Node next;
        next.lonDeg = from.lonDeg + (sample.lonDeg - from.lonDeg) * t;
        next.latDeg = from.latDeg + (sample.latDeg - from.latDeg) * t;
        next.altMeters = from.altMeters + (sample.altMeters - from.altMeters) * t;
        next.altMeters = std::max(next.altMeters, syntheticTerrainHeight(next.lonDeg, next.latDeg) + m_options.safetyClearanceMeters);
        next.parentIndex = nearest;

        if (!pointSafe(next.lonDeg, next.latDeg, next.altMeters, request, m_options)) {
            continue;
        }
        if (!segmentSafe(from, next, request, m_options)) {
            continue;
        }

        nodes.push_back(next);
        const int nextIndex = static_cast<int>(nodes.size()) - 1;

        const double toGoal = distance3dMeters(next.lonDeg, next.latDeg, next.altMeters, goal.lonDeg, goal.latDeg, goal.altMeters);
        if (toGoal <= std::max(1000.0, m_options.connectDistanceMeters)) {
            Node goalNode = goal;
            goalNode.parentIndex = nextIndex;
            if (segmentSafe(next, goalNode, request, m_options)) {
                nodes.push_back(goalNode);
                reachedIndex = static_cast<int>(nodes.size()) - 1;
                break;
            }
        }

        result.metrics.expandedNodes = static_cast<int>(nodes.size());
    }

    if (reachedIndex < 0) {
        result.metrics.success = false;
        result.metrics.message = "RRT未在限制迭代内找到可行路径。";
        result.metrics.visitedNodes = static_cast<int>(nodes.size());
        result.metrics.planningTimeMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
        return result;
    }

    std::vector<osgEarth::GeoPoint> reversed;
    for (int i = reachedIndex; i >= 0; i = nodes[static_cast<std::size_t>(i)].parentIndex) {
        const auto& n = nodes[static_cast<std::size_t>(i)];
        reversed.emplace_back(wgs84, n.lonDeg, n.latDeg, n.altMeters, osgEarth::ALTMODE_ABSOLUTE);
        if (nodes[static_cast<std::size_t>(i)].parentIndex < 0) {
            break;
        }
    }

    result.route.reserve(reversed.size());
    for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
        result.route.push_back(*it);
    }

    if (!result.route.empty()) {
        result.route.front() = osgEarth::GeoPoint(wgs84, request.start.x(), request.start.y(), startAlt, osgEarth::ALTMODE_ABSOLUTE);
        result.route.back() = osgEarth::GeoPoint(wgs84, request.goal.x(), request.goal.y(), goalAlt, osgEarth::ALTMODE_ABSOLUTE);
        result.route = shortcutSmooth(result.route, request, m_options);
        result.route.front() = osgEarth::GeoPoint(wgs84, request.start.x(), request.start.y(), startAlt, osgEarth::ALTMODE_ABSOLUTE);
        result.route.back() = osgEarth::GeoPoint(wgs84, request.goal.x(), request.goal.y(), goalAlt, osgEarth::ALTMODE_ABSOLUTE);

        for (int fixPass = 0; fixPass < 4; ++fixPass) {
            bool anyUnsafe = false;
            for (std::size_t i = 1; i + 1 < result.route.size(); ++i) {
                const auto& p = result.route[i];
                if (pointSafe(p.x(), p.y(), p.z(), request, m_options)) {
                    continue;
                }
                anyUnsafe = true;
                bool fixed = false;
                for (double altUp = p.z() + 500.0; altUp <= 95000.0; altUp += 500.0) {
                    if (pointSafe(p.x(), p.y(), altUp, request, m_options)) {
                        result.route[i] = osgEarth::GeoPoint(wgs84, p.x(), p.y(), altUp, osgEarth::ALTMODE_ABSOLUTE);
                        fixed = true;
                        break;
                    }
                }
                if (!fixed) {
                    for (double altDown = p.z() - 500.0; altDown >= 0.0; altDown -= 500.0) {
                        if (pointSafe(p.x(), p.y(), altDown, request, m_options)) {
                            result.route[i] = osgEarth::GeoPoint(wgs84, p.x(), p.y(), altDown, osgEarth::ALTMODE_ABSOLUTE);
                            fixed = true;
                            break;
                        }
                    }
                }
            }
            if (!anyUnsafe) {
                break;
            }
        }
    }

    result.metrics.success = true;
    result.metrics.message = "RRT规划成功。";
    result.metrics.pathLengthMeters = routeLengthMeters(result.route);
    result.metrics.expandedNodes = static_cast<int>(nodes.size());
    result.metrics.visitedNodes = static_cast<int>(nodes.size());
    result.metrics.planningTimeMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();

    return result;
}

}  // namespace mission
