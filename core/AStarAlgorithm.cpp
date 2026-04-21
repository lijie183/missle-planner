#include "core/AStarAlgorithm.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <osgEarth/SpatialReference>

namespace {

constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kMetersPerLatDegree = 111320.0;

struct SearchBounds {
    double minLonDeg = 0.0;
    double maxLonDeg = 0.0;
    double minLatDeg = 0.0;
    double maxLatDeg = 0.0;
    double minAltMeters = 0.0;
    double maxAltMeters = 0.0;
    double lonStepDeg = 0.05;
    double latStepDeg = 0.05;
    double altStepMeters = 250.0;
    int sizeX = 0;
    int sizeY = 0;
    int sizeZ = 0;
};

struct GridKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHasher {
    std::size_t operator()(const GridKey& key) const {
        const auto hx = static_cast<std::size_t>(key.x) * 73856093u;
        const auto hy = static_cast<std::size_t>(key.y) * 19349663u;
        const auto hz = static_cast<std::size_t>(key.z) * 83492791u;
        return hx ^ hy ^ hz;
    }
};

struct NodeRecord {
    double gCost = std::numeric_limits<double>::infinity();
    double fCost = std::numeric_limits<double>::infinity();
    GridKey parent{};
    bool hasParent = false;
    bool closed = false;
};

struct OpenEntry {
    GridKey key{};
    double fCost = 0.0;

    bool operator<(const OpenEntry& other) const {
        return fCost > other.fCost;
    }
};

double toRadians(double degrees) {
    return degrees * 0.017453292519943295;
}

double approxHorizontalDistanceMeters(double lonDegA, double latDegA, double lonDegB, double latDegB) {
    const double meanLatRad = toRadians((latDegA + latDegB) * 0.5);
    const double metersPerLonDegree = kMetersPerLatDegree * std::max(0.1, std::cos(meanLatRad));
    const double dx = (lonDegB - lonDegA) * metersPerLonDegree;
    const double dy = (latDegB - latDegA) * kMetersPerLatDegree;
    return std::sqrt(dx * dx + dy * dy);
}

double approx3dDistanceMeters(
    double lonDegA,
    double latDegA,
    double altMetersA,
    double lonDegB,
    double latDegB,
    double altMetersB) {
    const double horizontal = approxHorizontalDistanceMeters(lonDegA, latDegA, lonDegB, latDegB);
    const double dz = altMetersB - altMetersA;
    return std::sqrt(horizontal * horizontal + dz * dz);
}

double terrainHeightMeters(double lonDeg, double latDeg) {
    // 使用平滑的合成地形，在离线情况下也能驱动地形规避策略。
    const double latWave = std::sin(toRadians(latDeg * 4.0));
    const double lonWave = std::cos(toRadians(lonDeg * 3.0));
    const double detailWave = std::sin(toRadians((lonDeg + latDeg) * 8.0));
    return 260.0 + 180.0 * latWave + 120.0 * lonWave + 60.0 * detailWave;
}

int clampToIndex(double value, double minValue, double step, int size) {
    if (size <= 1 || step <= 0.0) {
        return 0;
    }
    const int index = static_cast<int>(std::llround((value - minValue) / step));
    return std::clamp(index, 0, size - 1);
}

double keyToLonDeg(const GridKey& key, const SearchBounds& bounds) {
    return bounds.minLonDeg + static_cast<double>(key.x) * bounds.lonStepDeg;
}

double keyToLatDeg(const GridKey& key, const SearchBounds& bounds) {
    return bounds.minLatDeg + static_cast<double>(key.y) * bounds.latStepDeg;
}

double keyToAltMeters(const GridKey& key, const SearchBounds& bounds) {
    return bounds.minAltMeters + static_cast<double>(key.z) * bounds.altStepMeters;
}

SearchBounds buildSearchBounds(const mission::MissionRequest& request, const mission::AStarAlgorithm::Options& options) {
    SearchBounds bounds;

    double minLon = std::min(request.start.x(), request.goal.x());
    double maxLon = std::max(request.start.x(), request.goal.x());
    double minLat = std::min(request.start.y(), request.goal.y());
    double maxLat = std::max(request.start.y(), request.goal.y());

    const double referenceLatRad = toRadians((request.start.y() + request.goal.y()) * 0.5);
    const double metersPerLonDegree = kMetersPerLatDegree * std::max(0.1, std::cos(referenceLatRad));

    double highestThreatAltitude = std::max(request.start.z(), request.goal.z());
    double maxThreatRadiusMeters = 0.0;
    for (const auto& threat : request.threats) {
        const double lonRadiusDeg = threat.radiusMeters / metersPerLonDegree;
        const double latRadiusDeg = threat.radiusMeters / kMetersPerLatDegree;

        minLon = std::min(minLon, threat.longitudeDeg - lonRadiusDeg - 0.05);
        maxLon = std::max(maxLon, threat.longitudeDeg + lonRadiusDeg + 0.05);
        minLat = std::min(minLat, threat.latitudeDeg - latRadiusDeg - 0.05);
        maxLat = std::max(maxLat, threat.latitudeDeg + latRadiusDeg + 0.05);

        highestThreatAltitude = std::max(highestThreatAltitude, threat.maxAltitudeMeters);
        maxThreatRadiusMeters = std::max(maxThreatRadiusMeters, threat.radiusMeters);
    }

    const double threatMarginDeg = std::max(
        options.searchMarginDeg,
        (maxThreatRadiusMeters * 2.0) / kMetersPerLatDegree + 0.1);

    minLon -= threatMarginDeg;
    maxLon += threatMarginDeg;
    minLat -= threatMarginDeg;
    maxLat += threatMarginDeg;

    bounds.minLonDeg = minLon;
    bounds.maxLonDeg = maxLon;
    bounds.minLatDeg = minLat;
    bounds.maxLatDeg = maxLat;

    double stepDeg = std::max(0.01, options.gridStepDeg);
    int nx = static_cast<int>(std::ceil((maxLon - minLon) / stepDeg)) + 1;
    int ny = static_cast<int>(std::ceil((maxLat - minLat) / stepDeg)) + 1;
    while (nx * ny > 11000) {
        stepDeg *= 1.2;
        nx = static_cast<int>(std::ceil((maxLon - minLon) / stepDeg)) + 1;
        ny = static_cast<int>(std::ceil((maxLat - minLat) / stepDeg)) + 1;
    }

    bounds.lonStepDeg = stepDeg;
    bounds.latStepDeg = stepDeg;
    bounds.sizeX = std::max(3, nx);
    bounds.sizeY = std::max(3, ny);

    double sampledTerrainMax = terrainHeightMeters(request.start.x(), request.start.y());
    sampledTerrainMax = std::max(sampledTerrainMax, terrainHeightMeters(request.goal.x(), request.goal.y()));
    const int sampleGrid = 8;
    for (int ix = 0; ix <= sampleGrid; ++ix) {
        for (int iy = 0; iy <= sampleGrid; ++iy) {
            const double lon = minLon + (maxLon - minLon) * (static_cast<double>(ix) / sampleGrid);
            const double lat = minLat + (maxLat - minLat) * (static_cast<double>(iy) / sampleGrid);
            sampledTerrainMax = std::max(sampledTerrainMax, terrainHeightMeters(lon, lat));
        }
    }

    bounds.minAltMeters = std::max(0.0, std::min(request.start.z(), request.goal.z()) - 400.0);
    bounds.maxAltMeters = std::max(
        {request.start.z(), request.goal.z(), highestThreatAltitude + 900.0, sampledTerrainMax + 1800.0});

    double altStep = std::max(80.0, options.altitudeStepMeters);
    int nz = static_cast<int>(std::ceil((bounds.maxAltMeters - bounds.minAltMeters) / altStep)) + 1;
    while (nz > std::max(4, options.maxAltitudeLevels)) {
        altStep *= 1.25;
        nz = static_cast<int>(std::ceil((bounds.maxAltMeters - bounds.minAltMeters) / altStep)) + 1;
    }

    bounds.altStepMeters = altStep;
    bounds.sizeZ = std::max(4, nz);

    return bounds;
}

bool inThreatZone(
    double lonDeg,
    double latDeg,
    double altMeters,
    const mission::ThreatZone& threat,
    double altitudeSafetyMargin) {
    const double horizontal = approxHorizontalDistanceMeters(
        lonDeg,
        latDeg,
        threat.longitudeDeg,
        threat.latitudeDeg);

    if (horizontal > threat.radiusMeters) {
        return false;
    }

    return altMeters >= (threat.minAltitudeMeters - altitudeSafetyMargin) &&
           altMeters <= (threat.maxAltitudeMeters + altitudeSafetyMargin);
}

double threatProximityPenalty(
    double lonDeg,
    double latDeg,
    double altMeters,
    const std::vector<mission::ThreatZone>& threats,
    double penaltyScale) {
    double penalty = 0.0;

    for (const auto& threat : threats) {
        const double horizontal = approxHorizontalDistanceMeters(
            lonDeg,
            latDeg,
            threat.longitudeDeg,
            threat.latitudeDeg);

        const double influenceRadius = threat.radiusMeters * 1.6;
        const bool altitudeRelevant = altMeters <= (threat.maxAltitudeMeters + 1000.0);
        if (!altitudeRelevant || horizontal >= influenceRadius) {
            continue;
        }

        if (horizontal <= threat.radiusMeters) {
            return std::numeric_limits<double>::infinity();
        }

        const double normalized = (influenceRadius - horizontal) / std::max(1.0, influenceRadius - threat.radiusMeters);
        penalty += penaltyScale * normalized;
    }

    return penalty;
}

bool isValidState(
    const GridKey& key,
    const SearchBounds& bounds,
    const mission::MissionRequest& request,
    const mission::AStarAlgorithm::Options& options) {
    const double lonDeg = keyToLonDeg(key, bounds);
    const double latDeg = keyToLatDeg(key, bounds);
    const double altMeters = keyToAltMeters(key, bounds);

    const double terrain = terrainHeightMeters(lonDeg, latDeg);
    if (altMeters < terrain + options.safetyClearanceMeters) {
        return false;
    }

    for (const auto& threat : request.threats) {
        if (inThreatZone(lonDeg, latDeg, altMeters, threat, 0.0)) {
            return false;
        }
    }

    return true;
}

GridKey findNearestValidKey(
    const GridKey& desired,
    const SearchBounds& bounds,
    const mission::MissionRequest& request,
    const mission::AStarAlgorithm::Options& options) {
    GridKey candidate = desired;
    candidate.z = std::clamp(candidate.z, 0, bounds.sizeZ - 1);

    for (int dz = 0; dz < bounds.sizeZ; ++dz) {
        const int upZ = std::clamp(desired.z + dz, 0, bounds.sizeZ - 1);
        GridKey up{desired.x, desired.y, upZ};
        if (isValidState(up, bounds, request, options)) {
            return up;
        }

        const int downZ = std::clamp(desired.z - dz, 0, bounds.sizeZ - 1);
        GridKey down{desired.x, desired.y, downZ};
        if (isValidState(down, bounds, request, options)) {
            return down;
        }
    }

    return candidate;
}

std::vector<GridKey> neighborOffsets() {
    std::vector<GridKey> offsets;
    offsets.reserve(26);

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                offsets.push_back({dx, dy, dz});
            }
        }
    }

    return offsets;
}

double routeLengthMeters(const std::vector<osgEarth::GeoPoint>& route) {
    if (route.size() < 2) {
        return 0.0;
    }

    double total = 0.0;
    for (std::size_t i = 1; i < route.size(); ++i) {
        const auto& a = route[i - 1];
        const auto& b = route[i];
        total += approx3dDistanceMeters(a.x(), a.y(), a.z(), b.x(), b.y(), b.z());
    }

    return total;
}

}  // namespace

namespace mission {

AStarAlgorithm::AStarAlgorithm() = default;

AStarAlgorithm::AStarAlgorithm(const Options& options)
    : m_options(options) {}

void AStarAlgorithm::setOptions(const Options& options) {
    m_options = options;
}

const AStarAlgorithm::Options& AStarAlgorithm::options() const {
    return m_options;
}

RoutePlanResult AStarAlgorithm::plan(const MissionRequest& request) const {
    RoutePlanResult result;

    const auto startTime = std::chrono::steady_clock::now();
    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (wgs84 == nullptr) {
        result.metrics.message = "无法获取WGS84坐标参考。";
        return result;
    }

    const SearchBounds bounds = buildSearchBounds(request, m_options);
    const auto offsets = neighborOffsets();

    const GridKey desiredStart{
        clampToIndex(request.start.x(), bounds.minLonDeg, bounds.lonStepDeg, bounds.sizeX),
        clampToIndex(request.start.y(), bounds.minLatDeg, bounds.latStepDeg, bounds.sizeY),
        clampToIndex(request.start.z(), bounds.minAltMeters, bounds.altStepMeters, bounds.sizeZ)};

    const GridKey desiredGoal{
        clampToIndex(request.goal.x(), bounds.minLonDeg, bounds.lonStepDeg, bounds.sizeX),
        clampToIndex(request.goal.y(), bounds.minLatDeg, bounds.latStepDeg, bounds.sizeY),
        clampToIndex(request.goal.z(), bounds.minAltMeters, bounds.altStepMeters, bounds.sizeZ)};

    const GridKey startKey = findNearestValidKey(desiredStart, bounds, request, m_options);
    const GridKey goalKey = findNearestValidKey(desiredGoal, bounds, request, m_options);

    if (!isValidState(startKey, bounds, request, m_options)) {
        result.metrics.message = "起点不可达：起点位于地形下方或威胁区内部。";
        return result;
    }

    if (!isValidState(goalKey, bounds, request, m_options)) {
        result.metrics.message = "终点不可达：终点位于地形下方或威胁区内部。";
        return result;
    }

    std::unordered_map<GridKey, NodeRecord, GridKeyHasher> records;
    records.reserve(40000);

    auto heuristic = [&](const GridKey& key) {
        return approx3dDistanceMeters(
            keyToLonDeg(key, bounds),
            keyToLatDeg(key, bounds),
            keyToAltMeters(key, bounds),
            keyToLonDeg(goalKey, bounds),
            keyToLatDeg(goalKey, bounds),
            keyToAltMeters(goalKey, bounds));
    };

    std::priority_queue<OpenEntry> openSet;

    NodeRecord& startRecord = records[startKey];
    startRecord.gCost = 0.0;
    startRecord.fCost = heuristic(startKey);
    openSet.push({startKey, startRecord.fCost});

    int iterations = 0;
    int expandedNodes = 0;
    bool reachedGoal = false;
    GridKey finalKey = startKey;

    while (!openSet.empty() && iterations < m_options.maxIterations) {
        ++iterations;

        const OpenEntry currentEntry = openSet.top();
        openSet.pop();

        auto currentIt = records.find(currentEntry.key);
        if (currentIt == records.end()) {
            continue;
        }

        NodeRecord& currentRecord = currentIt->second;
        if (currentRecord.closed) {
            continue;
        }
        if (currentEntry.fCost > currentRecord.fCost + 1e-6) {
            continue;
        }

        currentRecord.closed = true;
        ++expandedNodes;

        if (currentEntry.key == goalKey) {
            reachedGoal = true;
            finalKey = currentEntry.key;
            break;
        }

        for (const auto& offset : offsets) {
            GridKey next{
                currentEntry.key.x + offset.x,
                currentEntry.key.y + offset.y,
                currentEntry.key.z + offset.z};

            if (next.x < 0 || next.x >= bounds.sizeX ||
                next.y < 0 || next.y >= bounds.sizeY ||
                next.z < 0 || next.z >= bounds.sizeZ) {
                continue;
            }

            if (!isValidState(next, bounds, request, m_options)) {
                continue;
            }

            const double currentLon = keyToLonDeg(currentEntry.key, bounds);
            const double currentLat = keyToLatDeg(currentEntry.key, bounds);
            const double currentAlt = keyToAltMeters(currentEntry.key, bounds);

            const double nextLon = keyToLonDeg(next, bounds);
            const double nextLat = keyToLatDeg(next, bounds);
            const double nextAlt = keyToAltMeters(next, bounds);

            double edgeCost = approx3dDistanceMeters(
                currentLon,
                currentLat,
                currentAlt,
                nextLon,
                nextLat,
                nextAlt);

            edgeCost += std::abs(nextAlt - currentAlt) * 0.2;

            const double proximityPenalty = threatProximityPenalty(
                nextLon,
                nextLat,
                nextAlt,
                request.threats,
                m_options.threatPenaltyScale);
            if (!std::isfinite(proximityPenalty)) {
                continue;
            }

            edgeCost += proximityPenalty;

            const double tentativeG = currentRecord.gCost + edgeCost;

            NodeRecord& nextRecord = records[next];
            if (nextRecord.closed && tentativeG >= nextRecord.gCost) {
                continue;
            }

            if (tentativeG + 1e-6 < nextRecord.gCost) {
                nextRecord.gCost = tentativeG;
                nextRecord.fCost = tentativeG + heuristic(next);
                nextRecord.parent = currentEntry.key;
                nextRecord.hasParent = true;
                nextRecord.closed = false;
                openSet.push({next, nextRecord.fCost});
            }
        }
    }

    if (!reachedGoal) {
        result.metrics.success = false;
        result.metrics.message = "A*未在限制迭代内找到可行路径。";
        result.metrics.expandedNodes = expandedNodes;
        result.metrics.visitedNodes = static_cast<int>(records.size());
        result.metrics.planningTimeMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();
        return result;
    }

    std::vector<GridKey> reversedPath;
    reversedPath.reserve(1024);

    GridKey walk = finalKey;
    reversedPath.push_back(walk);

    while (!(walk == startKey)) {
        const auto it = records.find(walk);
        if (it == records.end() || !it->second.hasParent) {
            break;
        }
        walk = it->second.parent;
        reversedPath.push_back(walk);
    }

    if (!(reversedPath.back() == startKey)) {
        reversedPath.push_back(startKey);
    }

    result.route.reserve(reversedPath.size());
    for (auto it = reversedPath.rbegin(); it != reversedPath.rend(); ++it) {
        result.route.emplace_back(
            wgs84,
            keyToLonDeg(*it, bounds),
            keyToLatDeg(*it, bounds),
            keyToAltMeters(*it, bounds),
            osgEarth::ALTMODE_ABSOLUTE);
    }

    if (!result.route.empty()) {
        result.route.front() = osgEarth::GeoPoint(
            wgs84,
            request.start.x(),
            request.start.y(),
            std::max(request.start.z(), terrainHeightMeters(request.start.x(), request.start.y()) + m_options.safetyClearanceMeters),
            osgEarth::ALTMODE_ABSOLUTE);

        result.route.back() = osgEarth::GeoPoint(
            wgs84,
            request.goal.x(),
            request.goal.y(),
            std::max(request.goal.z(), terrainHeightMeters(request.goal.x(), request.goal.y()) + m_options.safetyClearanceMeters),
            osgEarth::ALTMODE_ABSOLUTE);
    }

    result.metrics.success = true;
    result.metrics.message = "A*规划成功。";
    result.metrics.expandedNodes = expandedNodes;
    result.metrics.visitedNodes = static_cast<int>(records.size());
    result.metrics.pathLengthMeters = routeLengthMeters(result.route);
    result.metrics.planningTimeMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();

    return result;
}

}  // namespace mission
