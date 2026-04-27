#pragma once

#include <QMainWindow>
#include <QElapsedTimer>
#include <QTimer>

#include <map>
#include <memory>
#include <vector>

#include <osgEarth/GeoData>

#include "core/MissionTypes.h"
#include "core/MultiMissilePlanner.h"
#include "sim/MissileSim.h"

class QLabel;
class QListWidget;
class QDoubleSpinBox;
class QCheckBox;
class QComboBox;
class QSpinBox;
class QTableWidget;
class QPushButton;

class OsgEarthWidget;
class TelemetryPlotWidget;

struct MissileRuntime {
    mission::MissileConfig config;
    mission::MissileSim sim;
    std::vector<osgEarth::GeoPoint> route;
    std::vector<mission::TelemetrySample> telemetryHistory;
    bool active = false;
    bool failed = false;
    bool completed = false;
    bool hasTelemetryPrevPoint = false;
    osgEarth::GeoPoint prevTelemetryPoint;
    double prevTelemetrySpeed = 0.0;
    int assignedTargetIndex = -1;
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

private slots:
    void onAddMissile();
    void onRemoveMissile();
    void onMissileSelectionChanged();
    void onAddTarget();
    void onRemoveTarget();
    void onTargetSelectionChanged();
    void onAddThreat();
    void onClearThreats();
    void onPlanRoute();
    void onStartSimulation();
    void onSimulationTick();
    void onSimulateFailure();
    void onDynamicReplan();

private:
    void buildUi();
    osgEarth::GeoPoint makeGeoPoint(
        const QDoubleSpinBox* lonSpin,
        const QDoubleSpinBox* latSpin,
        const QDoubleSpinBox* altSpin) const;
    void refreshThreatList();
    void refreshMissileList();
    void refreshTargetList();
    void buildMissileLegend();
    void refreshMetrics(const mission::PlanMetrics& metrics);
    void refreshSceneDataSourceLabel();
    void resetTelemetryPanel();
    void setExportEnabled(bool enabled);
    void onExportHtmlReport();
    void updateTelemetryPanel(
        int missileIndex,
        const osgEarth::GeoPoint& missilePoint,
        const mission::MissileSim::State& simState,
        double realDeltaSeconds);
    void updateAssignmentTable();
    void updateOverallMetrics();
    void updateScenarioSummary();
    void saveCurrentMissileParams();
    void saveCurrentTargetParams();
    void loadMissileParams(int index);
    void loadTargetParams(int index);
    void syncEarthWidgetFromConfig();
    void stopAllSimulations();

    OsgEarthWidget* m_earthWidget = nullptr;
    TelemetryPlotWidget* m_telemetryWidget = nullptr;
    QWidget* m_missileLegend = nullptr;

    QListWidget* m_missileList = nullptr;
    QDoubleSpinBox* m_missileLon = nullptr;
    QDoubleSpinBox* m_missileLat = nullptr;
    QDoubleSpinBox* m_missileAlt = nullptr;
    QComboBox* m_missileTypeCombo = nullptr;
    QDoubleSpinBox* m_missileSpeedSpin = nullptr;

    QListWidget* m_targetList = nullptr;
    QDoubleSpinBox* m_targetLon = nullptr;
    QDoubleSpinBox* m_targetLat = nullptr;
    QDoubleSpinBox* m_targetAlt = nullptr;
    QSpinBox* m_targetPrioritySpin = nullptr;

    QDoubleSpinBox* m_threatLon = nullptr;
    QDoubleSpinBox* m_threatLat = nullptr;
    QDoubleSpinBox* m_threatRadius = nullptr;
    QDoubleSpinBox* m_threatMaxAlt = nullptr;

    QComboBox* m_allocationCombo = nullptr;
    QComboBox* m_profileCombo = nullptr;
    QDoubleSpinBox* m_clearanceSpin = nullptr;
    QDoubleSpinBox* m_gridStepSpin = nullptr;
    QCheckBox* m_threatPenaltyCheck = nullptr;

    QDoubleSpinBox* m_timeScaleSpin = nullptr;
    QCheckBox* m_followMissileCheck = nullptr;
    QComboBox* m_globeModeCombo = nullptr;

    QListWidget* m_threatList = nullptr;
    QTableWidget* m_assignmentTable = nullptr;
    QComboBox* m_failureMissileCombo = nullptr;

    QLabel* m_sceneDataSourceValue = nullptr;
    QLabel* m_blueForceCountValue = nullptr;
    QLabel* m_redTargetCountValue = nullptr;
    QLabel* m_threatZoneCountValue = nullptr;
    QLabel* m_scenarioBalanceValue = nullptr;
    QLabel* m_planHealthValue = nullptr;

    QLabel* m_planTimeValue = nullptr;
    QLabel* m_pathLengthValue = nullptr;
    QLabel* m_nodesValue = nullptr;
    QLabel* m_simProgressValue = nullptr;
    QLabel* m_etaValue = nullptr;
    QLabel* m_phaseValue = nullptr;
    QLabel* m_currentSpeedValue = nullptr;
    QLabel* m_successRateValue = nullptr;
    QLabel* m_successCountValue = nullptr;
    QLabel* m_failureCountValue = nullptr;
    QLabel* m_totalTargetsValue = nullptr;
    QPushButton* m_exportHtmlButton = nullptr;

    QTimer m_simulationTimer;
    QElapsedTimer m_tickClock;
    qint64 m_lastTickMs = 0;

    std::vector<mission::MissileConfig> m_missileConfigs;
    std::vector<mission::TargetConfig> m_targetConfigs;
    std::vector<mission::ThreatZone> m_threatZones;
    std::vector<MissileRuntime> m_missileRuntimes;
    mission::MultiMissionResult m_lastMultiResult;

    int m_selectedMissileIndex = -1;
    int m_selectedTargetIndex = -1;
    int m_nextMissileId = 1;
    int m_nextTargetId = 1;
};
