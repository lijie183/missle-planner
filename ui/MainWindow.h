#pragma once

#include <QMainWindow>
#include <QElapsedTimer>
#include <QTimer>

#include <vector>

#include <osgEarth/GeoData>

#include "core/MissionTypes.h"
#include "core/RoutePlanner.h"
#include "sim/MissileSim.h"

class QLabel;
class QListWidget;
class QDoubleSpinBox;
class QCheckBox;
class QComboBox;

class OsgEarthWidget;
class TelemetryPlotWidget;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

private slots:
    void onAddThreat();
    void onClearThreats();
    void onPlanRoute();
    void onStartSimulation();
    void onSimulationTick();

private:
    void buildUi();
    osgEarth::GeoPoint makeGeoPoint(
        const QDoubleSpinBox* lonSpin,
        const QDoubleSpinBox* latSpin,
        const QDoubleSpinBox* altSpin) const;
    void refreshThreatList();
    void refreshMetrics(const mission::PlanMetrics& metrics);
    void refreshSceneDataSourceLabel();
    void resetTelemetryPanel();
    void updateTelemetryPanel(
        const osgEarth::GeoPoint& missilePoint,
        const mission::MissileSim::State& simState,
        double realDeltaSeconds);

    OsgEarthWidget* m_earthWidget = nullptr;
    TelemetryPlotWidget* m_telemetryWidget = nullptr;

    QDoubleSpinBox* m_startLon = nullptr;
    QDoubleSpinBox* m_startLat = nullptr;
    QDoubleSpinBox* m_startAlt = nullptr;

    QDoubleSpinBox* m_goalLon = nullptr;
    QDoubleSpinBox* m_goalLat = nullptr;
    QDoubleSpinBox* m_goalAlt = nullptr;

    QDoubleSpinBox* m_threatLon = nullptr;
    QDoubleSpinBox* m_threatLat = nullptr;
    QDoubleSpinBox* m_threatRadius = nullptr;
    QDoubleSpinBox* m_threatMaxAlt = nullptr;

    QComboBox* m_missileTypeCombo = nullptr;
    QComboBox* m_profileCombo = nullptr;
    QDoubleSpinBox* m_clearanceSpin = nullptr;
    QDoubleSpinBox* m_gridStepSpin = nullptr;
    QCheckBox* m_threatPenaltyCheck = nullptr;

    QDoubleSpinBox* m_speedSpin = nullptr;
    QDoubleSpinBox* m_timeScaleSpin = nullptr;
    QCheckBox* m_followMissileCheck = nullptr;
    QComboBox* m_globeModeCombo = nullptr;

    QListWidget* m_threatList = nullptr;

    QLabel* m_sceneDataSourceValue = nullptr;
    QLabel* m_planTimeValue = nullptr;
    QLabel* m_pathLengthValue = nullptr;
    QLabel* m_nodesValue = nullptr;
    QLabel* m_simProgressValue = nullptr;
    QLabel* m_etaValue = nullptr;
    QLabel* m_phaseValue = nullptr;
    QLabel* m_currentSpeedValue = nullptr;

    QTimer m_simulationTimer;
    QElapsedTimer m_tickClock;
    qint64 m_lastTickMs = 0;

    bool m_hasTelemetryPrevPoint = false;
    osgEarth::GeoPoint m_prevTelemetryPoint;
    double m_prevTelemetrySpeed = 0.0;

    mission::RoutePlanner m_routePlanner;
    mission::MissileSim m_missileSim;

    std::vector<mission::ThreatZone> m_threatZones;
    std::vector<osgEarth::GeoPoint> m_lastRoute;
};
