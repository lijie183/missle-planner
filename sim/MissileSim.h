#pragma once

#include <vector>

#include <osgEarth/GeoData>

namespace mission {

class MissileSim {
public:
    enum class Phase {
        Idle,
        Boost,
        Cruise,
        Terminal,
        Completed
    };

    struct State {
        bool running = false;
        double speedMetersPerSecond = 0.0;
        double currentSpeedMetersPerSecond = 0.0;
        double traveledMeters = 0.0;
        double totalMeters = 0.0;
        double elapsedSeconds = 0.0;
        Phase phase = Phase::Idle;
    };

    void setRoute(const std::vector<osgEarth::GeoPoint>& route);
    bool hasRoute() const;

    void start(double speedMetersPerSecond);
    void stop();

    bool isRunning() const;
    const State& state() const;

    bool update(double deltaSeconds, osgEarth::GeoPoint& outPosition);

private:
    Phase phaseForProgress(double progress) const;
    osgEarth::GeoPoint sampleByDistance(double traveledMeters) const;
    double desiredSpeedForPhase(Phase phase) const;
    double climbOffsetForProgress(double progress) const;

    std::vector<osgEarth::GeoPoint> m_route;
    std::vector<double> m_segmentLengths;
    std::vector<double> m_cumulativeLengths;
    State m_state;
    double m_launchClimbPeakMeters = 0.0;
};

}  // namespace mission
