#include "sim/MissileSim.h"

#include <algorithm>
#include <cmath>

#include <osgEarth/SpatialReference>

namespace {

constexpr double kMetersPerLatDegree = 111320.0;

double toRadians(double degrees) {
    return degrees * 0.017453292519943295;
}

double approx3dDistanceMeters(const osgEarth::GeoPoint& a, const osgEarth::GeoPoint& b) {
    const double meanLatRad = toRadians((a.y() + b.y()) * 0.5);
    const double metersPerLonDegree = kMetersPerLatDegree * std::max(0.1, std::cos(meanLatRad));

    const double dx = (b.x() - a.x()) * metersPerLonDegree;
    const double dy = (b.y() - a.y()) * kMetersPerLatDegree;
    const double dz = b.z() - a.z();

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

}  // namespace

namespace mission {

void MissileSim::setRoute(const std::vector<osgEarth::GeoPoint>& route) {
    m_route = route;
    m_segmentLengths.clear();
    m_cumulativeLengths.clear();

    m_state = {};

    if (m_route.size() < 2) {
        return;
    }

    m_segmentLengths.reserve(m_route.size() - 1);
    m_cumulativeLengths.reserve(m_route.size());

    m_cumulativeLengths.push_back(0.0);

    double cumulative = 0.0;
    for (std::size_t i = 1; i < m_route.size(); ++i) {
        const double seg = approx3dDistanceMeters(m_route[i - 1], m_route[i]);
        m_segmentLengths.push_back(seg);
        cumulative += seg;
        m_cumulativeLengths.push_back(cumulative);
    }

    m_state.totalMeters = cumulative;
}

bool MissileSim::hasRoute() const {
    return m_route.size() >= 2 && m_state.totalMeters > 0.0;
}

void MissileSim::start(double speedMetersPerSecond) {
    if (!hasRoute()) {
        m_state.running = false;
        return;
    }

    m_state.speedMetersPerSecond = std::max(1.0, speedMetersPerSecond);
    m_state.currentSpeedMetersPerSecond = m_state.speedMetersPerSecond;
    m_state.traveledMeters = 0.0;
    m_state.elapsedSeconds = 0.0;
    m_state.phase = Phase::Boost;
    m_state.running = true;
}

void MissileSim::stop() {
    m_state.running = false;
    if (m_state.totalMeters > 0.0 && m_state.traveledMeters >= m_state.totalMeters) {
        m_state.phase = Phase::Completed;
    }
}

bool MissileSim::isRunning() const {
    return m_state.running;
}

const MissileSim::State& MissileSim::state() const {
    return m_state;
}

bool MissileSim::update(double deltaSeconds, osgEarth::GeoPoint& outPosition) {
    if (!m_state.running || !hasRoute()) {
        return false;
    }

    const double safeDeltaSeconds = std::max(0.0, deltaSeconds);
    m_state.elapsedSeconds += safeDeltaSeconds;

    const double progress = m_state.totalMeters > 1e-6
                                ? std::clamp(m_state.traveledMeters / m_state.totalMeters, 0.0, 1.0)
                                : 0.0;

    m_state.phase = phaseForProgress(progress);

    double speedMultiplier = 1.0;
    switch (m_state.phase) {
        case Phase::Boost:
            speedMultiplier = 1.35;
            break;
        case Phase::Cruise:
            speedMultiplier = 1.0;
            break;
        case Phase::Terminal:
            speedMultiplier = 0.68;
            break;
        case Phase::Completed:
        case Phase::Idle:
        default:
            speedMultiplier = 1.0;
            break;
    }

    m_state.currentSpeedMetersPerSecond = m_state.speedMetersPerSecond * speedMultiplier;
    m_state.traveledMeters += safeDeltaSeconds * m_state.currentSpeedMetersPerSecond;
    if (m_state.traveledMeters >= m_state.totalMeters) {
        m_state.traveledMeters = m_state.totalMeters;
        m_state.running = false;
        m_state.phase = Phase::Completed;
        m_state.currentSpeedMetersPerSecond = 0.0;
    }

    outPosition = sampleByDistance(m_state.traveledMeters);
    return true;
}

MissileSim::Phase MissileSim::phaseForProgress(double progress) const {
    if (progress >= 1.0) {
        return Phase::Completed;
    }
    if (progress < 0.12) {
        return Phase::Boost;
    }
    if (progress < 0.85) {
        return Phase::Cruise;
    }
    return Phase::Terminal;
}

osgEarth::GeoPoint MissileSim::sampleByDistance(double traveledMeters) const {
    if (m_route.empty()) {
        return {};
    }

    if (m_route.size() == 1 || traveledMeters <= 0.0) {
        return m_route.front();
    }

    if (traveledMeters >= m_state.totalMeters) {
        return m_route.back();
    }

    auto upperIt = std::lower_bound(m_cumulativeLengths.begin(), m_cumulativeLengths.end(), traveledMeters);
    if (upperIt == m_cumulativeLengths.end()) {
        return m_route.back();
    }

    const std::size_t upperIndex = static_cast<std::size_t>(upperIt - m_cumulativeLengths.begin());
    if (upperIndex == 0) {
        return m_route.front();
    }

    const std::size_t lowerIndex = upperIndex - 1;

    const double lowerDist = m_cumulativeLengths[lowerIndex];
    const double upperDist = m_cumulativeLengths[upperIndex];
    const double segmentLen = std::max(1e-6, upperDist - lowerDist);
    const double t = std::clamp((traveledMeters - lowerDist) / segmentLen, 0.0, 1.0);

    const osgEarth::GeoPoint& a = m_route[lowerIndex];
    const osgEarth::GeoPoint& b = m_route[upperIndex];

    const auto* srs = a.getSRS();
    if (srs == nullptr) {
        srs = osgEarth::SpatialReference::get("wgs84");
    }

    return osgEarth::GeoPoint(
        srs,
        lerp(a.x(), b.x(), t),
        lerp(a.y(), b.y(), t),
        lerp(a.z(), b.z(), t),
        osgEarth::ALTMODE_ABSOLUTE);
}

}  // namespace mission
