#include "ui/MainWindow.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QDir>
#include <QFileDialog>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSaveFile>
#include <QSplitter>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

#include <osgEarth/SpatialReference>

#include "render/OsgEarthWidget.h"
#include "ui/FlightReportExporter.h"
#include "ui/TelemetryPlotWidget.h"

namespace {

QDoubleSpinBox* createSpinBox(
    double min,
    double max,
    double value,
    int decimals,
    double step,
    QWidget* parent) {
    auto* spin = new QDoubleSpinBox(parent);
    spin->setRange(min, max);
    spin->setValue(value);
    spin->setDecimals(decimals);
    spin->setSingleStep(step);
    return spin;
}

QString phaseToText(mission::MissileSim::Phase phase) {
    switch (phase) {
        case mission::MissileSim::Phase::Boost:
            return QStringLiteral("助推段");
        case mission::MissileSim::Phase::Cruise:
            return QStringLiteral("巡航段");
        case mission::MissileSim::Phase::Terminal:
            return QStringLiteral("末制导");
        case mission::MissileSim::Phase::Completed:
            return QStringLiteral("命中完成");
        case mission::MissileSim::Phase::Idle:
        default:
            return QStringLiteral("待命");
    }
}

double toRadians(double degrees) {
    return degrees * 0.017453292519943295;
}

double metersPerLonDegree(double latDeg) {
    constexpr double kMetersPerLatDegree = 111320.0;
    return kMetersPerLatDegree * std::max(0.1, std::cos(toRadians(latDeg)));
}

double horizontalDistanceMeters(const osgEarth::GeoPoint& a, const osgEarth::GeoPoint& b) {
    constexpr double kMetersPerLatDegree = 111320.0;
    const double meanLat = (a.y() + b.y()) * 0.5;
    const double dx = (b.x() - a.x()) * metersPerLonDegree(meanLat);
    const double dy = (b.y() - a.y()) * kMetersPerLatDegree;
    return std::sqrt(dx * dx + dy * dy);
}

double headingDegrees(const osgEarth::GeoPoint& from, const osgEarth::GeoPoint& to) {
    const double meanLat = (from.y() + to.y()) * 0.5;
    const double dx = (to.x() - from.x()) * metersPerLonDegree(meanLat);
    const double dy = (to.y() - from.y()) * 111320.0;

    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
        return 0.0;
    }

    double heading = std::atan2(dx, dy) * 57.29577951308232;
    if (heading < 0.0) {
        heading += 360.0;
    }
    return heading;
}

double pitchDegrees(const osgEarth::GeoPoint& from, const osgEarth::GeoPoint& to) {
    const double horizontal = horizontalDistanceMeters(from, to);
    const double dz = to.z() - from.z();
    if (horizontal < 1e-6 && std::abs(dz) < 1e-6) {
        return 0.0;
    }
    return std::atan2(dz, std::max(1e-3, horizontal)) * 57.29577951308232;
}

QString missileTypeName(int type) {
    switch (type) {
        case 0: return QStringLiteral("高亚音速");
        case 1: return QStringLiteral("超音速");
        case 2: return QStringLiteral("高超滑翔");
        default: return QStringLiteral("未知");
    }
}

double defaultSpeedForType(int type) {
    switch (type) {
        case 0: return 250.0;
        case 1: return 520.0;
        case 2: return 900.0;
        default: return 250.0;
    }
}

mission::AllocationMethod allocationMethodFromComboIndex(int index) {
    switch (index) {
        case 1:
            return mission::AllocationMethod::Hungarian;
        case 2:
            return mission::AllocationMethod::Genetic;
        case 3:
            return mission::AllocationMethod::Greedy;
        case 4:
            return mission::AllocationMethod::BalancedGreedy;
        default:
            return mission::AllocationMethod::Hungarian;
    }
}

QString allocationMethodName(mission::AllocationMethod method) {
    switch (method) {
        case mission::AllocationMethod::Hungarian:
            return QStringLiteral("匈牙利算法");
        case mission::AllocationMethod::Genetic:
            return QStringLiteral("遗传算法");
        case mission::AllocationMethod::Greedy:
            return QStringLiteral("贪心算法");
        case mission::AllocationMethod::BalancedGreedy:
            return QStringLiteral("优先级均衡贪心");
        default:
            return QStringLiteral("未知算法");
    }
}

double computeResultScore(const mission::MultiMissionResult& result) {
    double totalPathMeters = 0.0;
    int plannedCount = 0;
    for (const auto& a : result.assignments) {
        if (!a.planned) {
            continue;
        }
        totalPathMeters += a.planResult.metrics.pathLengthMeters;
        ++plannedCount;
    }

    const double avgPathKm = plannedCount > 0
                                 ? (totalPathMeters / static_cast<double>(plannedCount)) / 1000.0
                                 : 1e6;

    return result.successRate * 10000.0 +
           static_cast<double>(result.successCount) * 600.0 -
           static_cast<double>(result.failureCount) * 900.0 -
           result.totalPlanningTimeMs * 0.6 -
           avgPathKm * 12.0;
}

}  // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    buildUi();

    connect(&m_simulationTimer, &QTimer::timeout, this, &MainWindow::onSimulationTick);
    m_simulationTimer.setInterval(16);

    m_nextMissileId = 1;
    m_nextTargetId = 1;

    populateDefaultScenario();

    refreshMissileList();
    refreshTargetList();
    refreshThreatList();
    syncEarthWidgetFromConfig();
    if (m_earthWidget != nullptr) {
        m_earthWidget->setThreatZones(m_threatZones);
    }

    statusBar()->showMessage(QStringLiteral("系统就绪：请配置导弹与目标参数后执行多导弹规划。"));
}

void MainWindow::onAddMissile() {
    mission::MissileConfig cfg;
    cfg.id = QString("M%1").arg(m_nextMissileId).toStdString();
    cfg.name = QString("导弹-%1").arg(m_nextMissileId).toStdString();
    cfg.startLonDeg = m_missileLon->value();
    cfg.startLatDeg = m_missileLat->value();
    cfg.startAltMeters = m_missileAlt->value();
    cfg.missileType = m_missileTypeCombo->currentIndex();
    cfg.speedMps = m_missileSpeedSpin->value();
    ++m_nextMissileId;

    m_missileConfigs.push_back(cfg);
    refreshMissileList();
    syncEarthWidgetFromConfig();
    statusBar()->showMessage(QStringLiteral("已添加导弹。"), 2000);
}

void MainWindow::onRemoveMissile() {
    const int currentIdx = m_missileList->currentRow();
    if (currentIdx < 0 || currentIdx >= static_cast<int>(m_missileConfigs.size())) {
        return;
    }

    m_missileConfigs.erase(m_missileConfigs.begin() + currentIdx);
    refreshMissileList();
    syncEarthWidgetFromConfig();
    statusBar()->showMessage(QStringLiteral("已移除导弹。"), 2000);
}

void MainWindow::onMissileSelectionChanged() {
    saveCurrentMissileParams();

    const int row = m_missileList->currentRow();
    m_selectedMissileIndex = row;
    if (row >= 0 && row < static_cast<int>(m_missileConfigs.size())) {
        loadMissileParams(row);
    }
}

void MainWindow::onAddTarget() {
    mission::TargetConfig cfg;
    cfg.id = QString("T%1").arg(m_nextTargetId).toStdString();
    cfg.name = QString("目标-%1").arg(m_nextTargetId).toStdString();
    cfg.lonDeg = m_targetLon->value();
    cfg.latDeg = m_targetLat->value();
    cfg.altMeters = m_targetAlt->value();
    cfg.priority = static_cast<int>(m_targetPrioritySpin->value());
    ++m_nextTargetId;

    m_targetConfigs.push_back(cfg);
    refreshTargetList();
    syncEarthWidgetFromConfig();
    statusBar()->showMessage(QStringLiteral("已添加目标。"), 2000);
}

void MainWindow::onRemoveTarget() {
    const int currentIdx = m_targetList->currentRow();
    if (currentIdx < 0 || currentIdx >= static_cast<int>(m_targetConfigs.size())) {
        return;
    }

    m_targetConfigs.erase(m_targetConfigs.begin() + currentIdx);
    refreshTargetList();
    syncEarthWidgetFromConfig();
    statusBar()->showMessage(QStringLiteral("已移除目标。"), 2000);
}

void MainWindow::onTargetSelectionChanged() {
    saveCurrentTargetParams();

    const int row = m_targetList->currentRow();
    m_selectedTargetIndex = row;
    if (row >= 0 && row < static_cast<int>(m_targetConfigs.size())) {
        loadTargetParams(row);
    }
}

void MainWindow::onAddThreat() {
    mission::ThreatZone threat;
    threat.longitudeDeg = m_threatLon->value();
    threat.latitudeDeg = m_threatLat->value();
    threat.radiusMeters = m_threatRadius->value();
    threat.minAltitudeMeters = 0.0;
    threat.maxAltitudeMeters = m_threatMaxAlt->value();

    m_threatZones.push_back(threat);
    refreshThreatList();

    if (m_earthWidget != nullptr) {
        m_earthWidget->setThreatZones(m_threatZones);
    }

    statusBar()->showMessage(QStringLiteral("已添加威胁区。"), 2000);
}

void MainWindow::onClearThreats() {
    m_threatZones.clear();
    refreshThreatList();

    if (m_earthWidget != nullptr) {
        m_earthWidget->setThreatZones(m_threatZones);
    }

    statusBar()->showMessage(QStringLiteral("威胁区已清空。"), 2000);
}

void MainWindow::onPlanRoute() {
    saveCurrentMissileParams();
    saveCurrentTargetParams();

    if (m_missileConfigs.empty()) {
        QMessageBox::warning(this, QStringLiteral("配置错误"), QStringLiteral("请至少添加一个导弹。"));
        return;
    }
    if (m_targetConfigs.empty()) {
        QMessageBox::warning(this, QStringLiteral("配置错误"), QStringLiteral("请至少添加一个目标。"));
        return;
    }

    if (m_earthWidget != nullptr) {
        m_earthWidget->resetMissionGraphics();
        m_earthWidget->setThreatZones(m_threatZones);
    }

    mission::MultiMissilePlanner planner;
    mission::MultiMissilePlanner::Options options;
    options.astarOptions.gridStepDeg = m_gridStepSpin->value();
    options.astarOptions.safetyClearanceMeters = m_clearanceSpin->value();

    const QString profile = m_profileCombo->currentText();
    if (profile.contains(QStringLiteral("低空"))) {
        options.astarOptions.altitudeStepMeters = 160.0;
        options.astarOptions.threatPenaltyScale = 17000.0;
    } else if (profile.contains(QStringLiteral("高空"))) {
        options.astarOptions.altitudeStepMeters = 360.0;
        options.astarOptions.safetyClearanceMeters = std::max(options.astarOptions.safetyClearanceMeters, 520.0);
        options.astarOptions.threatPenaltyScale = 9500.0;
    } else {
        options.astarOptions.altitudeStepMeters = 230.0;
        options.astarOptions.threatPenaltyScale = 13000.0;
    }

    if (!m_threatPenaltyCheck->isChecked()) {
        options.astarOptions.threatPenaltyScale = 0.0;
    }

    const std::array<mission::AllocationMethod, 4> methods = {
        mission::AllocationMethod::Hungarian,
        mission::AllocationMethod::Genetic,
        mission::AllocationMethod::Greedy,
        mission::AllocationMethod::BalancedGreedy};

    m_algorithmComparisons.clear();
    m_algorithmComparisons.reserve(methods.size());

    for (const auto method : methods) {
        options.method = method;

        AlgorithmCompareItem item;
        item.method = method;
        item.name = allocationMethodName(method).toStdString();
        item.result = planner.plan(m_missileConfigs, m_targetConfigs, m_threatZones, options);
        item.score = computeResultScore(item.result);
        m_algorithmComparisons.push_back(item);
    }

    const int allocIndex = m_allocationCombo->currentIndex();
    const bool autoSelectBest = (allocIndex == 0);

    mission::AllocationMethod selectedMethod = allocationMethodFromComboIndex(allocIndex);
    int selectedIndex = -1;

    if (autoSelectBest) {
        double bestScore = -std::numeric_limits<double>::infinity();
        for (int i = 0; i < static_cast<int>(m_algorithmComparisons.size()); ++i) {
            if (m_algorithmComparisons[i].score > bestScore) {
                bestScore = m_algorithmComparisons[i].score;
                selectedIndex = i;
            }
        }
    } else {
        for (int i = 0; i < static_cast<int>(m_algorithmComparisons.size()); ++i) {
            if (m_algorithmComparisons[i].method == selectedMethod) {
                selectedIndex = i;
                break;
            }
        }
    }

    if (selectedIndex < 0) {
        selectedIndex = 0;
    }

    for (int i = 0; i < static_cast<int>(m_algorithmComparisons.size()); ++i) {
        m_algorithmComparisons[i].selected = (i == selectedIndex);
    }

    m_lastPlanningMethod = m_algorithmComparisons[selectedIndex].method;
    m_lastMultiResult = m_algorithmComparisons[selectedIndex].result;
    updateAlgorithmCompareTable();

    m_missileRuntimes.clear();
    m_missileRuntimes.resize(m_missileConfigs.size());
    for (std::size_t i = 0; i < m_missileConfigs.size(); ++i) {
        m_missileRuntimes[i].config = m_missileConfigs[i];
        m_missileRuntimes[i].telemetryHistory.clear();
    }

    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");

    if (m_earthWidget != nullptr) {
        m_earthWidget->setMissileCount(static_cast<int>(m_missileConfigs.size()));

        for (std::size_t i = 0; i < m_missileConfigs.size(); ++i) {
            const auto& mc = m_missileConfigs[i];
            if (wgs84 != nullptr) {
                osgEarth::GeoPoint startPoint(
                    wgs84, mc.startLonDeg, mc.startLatDeg, mc.startAltMeters, osgEarth::ALTMODE_ABSOLUTE);
                m_earthWidget->setMissileStartPoint(static_cast<int>(i), startPoint);
            }
        }

        for (const auto& assign : m_lastMultiResult.assignments) {
            if (assign.planned && assign.targetIndex >= 0 && wgs84 != nullptr) {
                const auto& tc = m_targetConfigs[assign.targetIndex];
                osgEarth::GeoPoint targetPoint(
                    wgs84, tc.lonDeg, tc.latDeg, tc.altMeters, osgEarth::ALTMODE_ABSOLUTE);
                m_earthWidget->setMissileTargetPoint(assign.missileIndex, targetPoint);
                m_earthWidget->setMissileRoute(assign.missileIndex, assign.planResult.route);

                m_missileRuntimes[assign.missileIndex].route = assign.planResult.route;
                m_missileRuntimes[assign.missileIndex].assignedTargetIndex = assign.targetIndex;
            }
        }

        m_earthWidget->focusOnAllRoutes();
    }

    updateAssignmentTable();
    updateOverallMetrics();
    setExportEnabled(false);

    if (!m_lastMultiResult.assignments.empty() && m_lastMultiResult.assignments.front().planned) {
        refreshMetrics(m_lastMultiResult.assignments.front().planResult.metrics);
    }

    refreshSceneDataSourceLabel();

    statusBar()->showMessage(
        QStringLiteral("多导弹规划完成(%1)：成功 %2 / 失败 %3，成功率 %4%")
            .arg(allocationMethodName(m_lastPlanningMethod))
            .arg(m_lastMultiResult.successCount)
            .arg(m_lastMultiResult.failureCount)
            .arg(m_lastMultiResult.successRate * 100.0, 0, 'f', 1),
        5000);
}

void MainWindow::onStartSimulation() {
    if (m_missileRuntimes.empty()) {
        onPlanRoute();
        if (m_missileRuntimes.empty()) {
            return;
        }
    }

    bool anyActive = false;
    for (std::size_t i = 0; i < m_missileRuntimes.size(); ++i) {
        auto& rt = m_missileRuntimes[i];
        if (rt.route.size() < 2 || rt.failed || rt.completed) {
            rt.active = false;
            continue;
        }

        rt.sim.setRoute(rt.route);
        rt.sim.start(rt.config.speedMps);
        rt.active = rt.sim.isRunning();
        if (rt.active) {
            anyActive = true;
        }
    }

    if (!anyActive) {
        QMessageBox::warning(this, QStringLiteral("推演失败"), QStringLiteral("没有可用的导弹航迹进行推演。"));
        return;
    }

    if (m_earthWidget != nullptr) {
        const bool follow = m_followMissileCheck != nullptr && m_followMissileCheck->isChecked();
        for (std::size_t i = 0; i < m_missileRuntimes.size(); ++i) {
            if (m_missileRuntimes[i].active && !m_missileRuntimes[i].route.empty()) {
                m_earthWidget->setMissilePosition(static_cast<int>(i), m_missileRuntimes[i].route.front());
                m_earthWidget->setFollowMissile(static_cast<int>(i), follow);
            }
        }
        if (!follow) {
            m_earthWidget->focusOnAllRoutes();
        }
    }

    m_tickClock.restart();
    m_lastTickMs = 0;

    resetTelemetryPanel();
    for (auto& rt : m_missileRuntimes) {
        rt.telemetryHistory.clear();
        rt.hasTelemetryPrevPoint = false;
        rt.prevTelemetrySpeed = 0.0;
    }
    setExportEnabled(false);

    m_failureMissileCombo->clear();
    for (std::size_t i = 0; i < m_missileRuntimes.size(); ++i) {
        if (m_missileRuntimes[i].active) {
            m_failureMissileCombo->addItem(
                QString::fromStdString(m_missileRuntimes[i].config.name),
                static_cast<int>(i));
        }
    }

    m_simulationTimer.start();
    statusBar()->showMessage(QStringLiteral("多导弹三维推演进行中..."));
}

void MainWindow::onSimulationTick() {
    if (!m_tickClock.isValid()) {
        m_tickClock.start();
        m_lastTickMs = 0;
    }

    const qint64 nowMs = m_tickClock.elapsed();
    const qint64 deltaMs = std::max<qint64>(1, nowMs - m_lastTickMs);
    m_lastTickMs = nowMs;

    const double realDeltaSeconds = static_cast<double>(deltaMs) / 1000.0;
    const double scaledDeltaSeconds = realDeltaSeconds * m_timeScaleSpin->value();

    bool anyRunning = false;
    int firstActiveIndex = -1;

    for (std::size_t i = 0; i < m_missileRuntimes.size(); ++i) {
        auto& rt = m_missileRuntimes[i];
        if (!rt.active || rt.failed || rt.completed) {
            continue;
        }

        osgEarth::GeoPoint missilePoint;
        const bool ok = rt.sim.update(scaledDeltaSeconds, missilePoint);
        if (!ok) {
            rt.active = false;
            if (rt.sim.state().phase == mission::MissileSim::Phase::Completed) {
                rt.completed = true;
            }

            if (m_earthWidget != nullptr && !rt.route.empty()) {
                m_earthWidget->setMissilePosition(static_cast<int>(i), rt.route.back());
                m_earthWidget->showMissileImpact(static_cast<int>(i), rt.route.back());
            }

            if (firstActiveIndex < 0) {
                firstActiveIndex = static_cast<int>(i);
            }
            continue;
        }

        updateTelemetryPanel(static_cast<int>(i), missilePoint, rt.sim.state(), realDeltaSeconds);
        if (m_earthWidget != nullptr) {
            m_earthWidget->setMissileTelemetry(
                static_cast<int>(i),
                rt.sim.state().currentSpeedMetersPerSecond,
                rt.sim.state().elapsedSeconds);
        }

        if (!rt.sim.isRunning()) {
            rt.active = false;
            rt.completed = true;

            if (m_earthWidget != nullptr && !rt.route.empty()) {
                m_earthWidget->setMissilePosition(static_cast<int>(i), rt.route.back());
                m_earthWidget->showMissileImpact(static_cast<int>(i), rt.route.back());
            }

            if (firstActiveIndex < 0) {
                firstActiveIndex = static_cast<int>(i);
            }
            continue;
        }

        anyRunning = true;
        if (m_earthWidget != nullptr) {
            m_earthWidget->setMissilePosition(static_cast<int>(i), missilePoint);
        }

        if (firstActiveIndex < 0) {
            firstActiveIndex = static_cast<int>(i);
        }
    }

    if (firstActiveIndex >= 0) {
        const auto& rt = m_missileRuntimes[firstActiveIndex];
        if (rt.active) {
            const auto& simState = rt.sim.state();
            const double progress = simState.totalMeters > 1e-6
                                        ? std::clamp(simState.traveledMeters / simState.totalMeters * 100.0, 0.0, 100.0)
                                        : 0.0;
            m_simProgressValue->setText(QStringLiteral("%1%").arg(progress, 0, 'f', 1));
            m_phaseValue->setText(phaseToText(simState.phase));
            m_currentSpeedValue->setText(QStringLiteral("%1").arg(simState.currentSpeedMetersPerSecond, 0, 'f', 1));
        }
    }

    updateOverallMetrics();

    if (!anyRunning) {
        m_simulationTimer.stop();

        int completed = 0;
        int failed = 0;
        for (const auto& rt : m_missileRuntimes) {
            if (rt.completed) ++completed;
            if (rt.failed) ++failed;
        }

        if (m_earthWidget != nullptr && firstActiveIndex >= 0) {
            const auto& rt = m_missileRuntimes[firstActiveIndex];
            if (!rt.route.empty()) {
                m_earthWidget->focusOnPoint(rt.route.back(), 90000.0);
            }
        }

        m_simProgressValue->setText(QStringLiteral("100.0%"));
        m_etaValue->setText(QStringLiteral("0.0"));
        m_phaseValue->setText(QStringLiteral("推演结束"));
        m_currentSpeedValue->setText(QStringLiteral("0.0"));

        statusBar()->showMessage(
            QStringLiteral("推演结束：命中 %1 / 失效 %2 / 总计 %3")
                .arg(completed)
                .arg(failed)
                .arg(static_cast<int>(m_missileRuntimes.size())),
            5000);

        setExportEnabled(true);
    }
}

void MainWindow::onSimulateFailure() {
    const int data = m_failureMissileCombo->currentData().toInt();
    if (data < 0 || data >= static_cast<int>(m_missileRuntimes.size())) {
        return;
    }

    auto& rt = m_missileRuntimes[data];
    if (!rt.active) {
        QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("该导弹未在飞行中。"));
        return;
    }

    rt.active = false;
    rt.failed = true;

    if (m_earthWidget != nullptr) {
        m_earthWidget->clearMissile(data);
    }

    m_failureMissileCombo->removeItem(m_failureMissileCombo->currentIndex());

    updateOverallMetrics();
    setExportEnabled(false);
    statusBar()->showMessage(
        QStringLiteral("导弹 %1 已失效。").arg(QString::fromStdString(rt.config.name)),
        3000);
}

void MainWindow::onDynamicReplan() {
    std::vector<int> failedIndices;
    std::vector<int> completedTargetIndices;

    for (std::size_t i = 0; i < m_missileRuntimes.size(); ++i) {
        if (m_missileRuntimes[i].failed) {
            failedIndices.push_back(static_cast<int>(i));
        }
        if (m_missileRuntimes[i].completed && m_missileRuntimes[i].assignedTargetIndex >= 0) {
            completedTargetIndices.push_back(m_missileRuntimes[i].assignedTargetIndex);
        }
    }

    if (failedIndices.empty()) {
        QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("当前无失效导弹，无需重规划。"));
        return;
    }

    mission::MultiMissilePlanner planner;
    mission::MultiMissilePlanner::Options options;

    const int allocIndex = m_allocationCombo->currentIndex();
    if (allocIndex == 0) {
        options.method = m_lastPlanningMethod;
    } else {
        options.method = allocationMethodFromComboIndex(allocIndex);
        m_lastPlanningMethod = options.method;
    }

    options.astarOptions.gridStepDeg = m_gridStepSpin->value();
    options.astarOptions.safetyClearanceMeters = m_clearanceSpin->value();

    const QString profile = m_profileCombo->currentText();
    if (profile.contains(QStringLiteral("低空"))) {
        options.astarOptions.altitudeStepMeters = 160.0;
        options.astarOptions.threatPenaltyScale = 17000.0;
    } else if (profile.contains(QStringLiteral("高空"))) {
        options.astarOptions.altitudeStepMeters = 360.0;
        options.astarOptions.safetyClearanceMeters = std::max(options.astarOptions.safetyClearanceMeters, 520.0);
        options.astarOptions.threatPenaltyScale = 9500.0;
    } else {
        options.astarOptions.altitudeStepMeters = 230.0;
        options.astarOptions.threatPenaltyScale = 13000.0;
    }

    if (!m_threatPenaltyCheck->isChecked()) {
        options.astarOptions.threatPenaltyScale = 0.0;
    }

    m_lastMultiResult = planner.replan(
        m_missileConfigs, m_targetConfigs, m_threatZones, options,
        failedIndices, completedTargetIndices);

    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");

    for (const auto& assign : m_lastMultiResult.assignments) {
        if (assign.planned && assign.targetIndex >= 0) {
            auto& rt = m_missileRuntimes[assign.missileIndex];
            rt.route = assign.planResult.route;
            rt.assignedTargetIndex = assign.targetIndex;
            rt.failed = false;
            rt.completed = false;
            rt.active = false;

            if (m_earthWidget != nullptr && wgs84 != nullptr) {
                const auto& tc = m_targetConfigs[assign.targetIndex];
                osgEarth::GeoPoint targetPoint(
                    wgs84, tc.lonDeg, tc.latDeg, tc.altMeters, osgEarth::ALTMODE_ABSOLUTE);
                m_earthWidget->setMissileTargetPoint(assign.missileIndex, targetPoint);
                m_earthWidget->setMissileRoute(assign.missileIndex, assign.planResult.route);
                m_earthWidget->clearMissileImpact(assign.missileIndex);
            }
        }
    }

    if (m_earthWidget != nullptr) {
        m_earthWidget->focusOnAllRoutes();
    }

    updateAssignmentTable();
    updateOverallMetrics();

    m_failureMissileCombo->clear();
    for (std::size_t i = 0; i < m_missileRuntimes.size(); ++i) {
        if (m_missileRuntimes[i].active) {
            m_failureMissileCombo->addItem(
                QString::fromStdString(m_missileRuntimes[i].config.name),
                static_cast<int>(i));
        }
    }

    statusBar()->showMessage(
        QStringLiteral("动态重规划完成：成功 %1 / 失败 %2")
            .arg(m_lastMultiResult.successCount)
            .arg(m_lastMultiResult.failureCount),
        5000);

    if (m_bestAlgoValue != nullptr) {
        m_bestAlgoValue->setText(allocationMethodName(m_lastPlanningMethod));
    }
}

void MainWindow::buildUi() {
    setWindowTitle(QStringLiteral("空面导弹多弹协同任务规划与验证可视化系统"));
    resize(1560, 920);

    setStyleSheet(QStringLiteral(
        "QMainWindow, QWidget { background: #0b1320; color: #d8e4f2; }"
        "#topBar { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0f1e33, stop:1 #162846); border: 1px solid #2c4f71; border-radius: 8px; }"
        "#brandLabel { color: #ecf6ff; font-size: 16px; font-weight: 700; letter-spacing: 1px; }"
        "#brandSubLabel { color: #89a8c5; font-size: 11px; }"
        "#navButton { background: transparent; border: 1px solid #335a7e; border-radius: 6px; padding: 7px 16px; color: #9fc5e9; font-weight: 600; }"
        "#navButton:hover:!checked { background: #193957; }"
        "#navButton:checked { background: #1f5f8f; border-color: #57a7e0; color: #f4faff; }"
        "QGroupBox { border: 1px solid #2a4058; border-radius: 6px; margin-top: 10px; padding-top: 8px; font-weight: 600; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 10px; color: #97cbf6; }"
        "QFrame { background: #111f31; border: 1px solid #2a425b; border-radius: 6px; }"
        "QLabel { color: #d8e4f2; }"
        "QPushButton { background: #1c79b4; border: 1px solid #3d97cf; border-radius: 5px; padding: 6px 10px; color: #f1f9ff; font-weight: 600; }"
        "QPushButton:hover { background: #2591d4; }"
        "QPushButton:pressed { background: #17689b; }"
        "QDoubleSpinBox, QSpinBox, QComboBox, QListWidget, QTableWidget { background: #0f1b28; border: 1px solid #375571; border-radius: 4px; color: #e8f3ff; selection-background-color: #2e78ad; }"
        "QListWidget::item { padding: 5px 4px; border-bottom: 1px solid rgba(72, 107, 139, 0.35); }"
        "QListWidget::item:selected { background: #1f5f8f; color: #f4faff; }"
        "QSplitter::handle { background: #16283d; border: 1px solid #2d4f70; }"
        "QTableWidget::item { padding: 3px; }"
        "QHeaderView::section { background: #13263b; color: #9ec2e4; border: 1px solid #2d455f; padding: 4px; }"
        "QStatusBar { background: #0a131f; color: #9ec2e4; }"));

    auto* central = new QWidget(this);
    auto* rootLayout = new QVBoxLayout(central);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* topBar = new QFrame(central);
    topBar->setObjectName(QStringLiteral("topBar"));
    auto* topBarLayout = new QHBoxLayout(topBar);
    topBarLayout->setContentsMargins(12, 8, 12, 8);
    topBarLayout->setSpacing(10);

    auto* brandWrap = new QWidget(topBar);
    auto* brandLayout = new QVBoxLayout(brandWrap);
    brandLayout->setContentsMargins(0, 0, 0, 0);
    brandLayout->setSpacing(2);
    auto* brandLabel = new QLabel(QStringLiteral("MPMP 空面导弹协同规划平台"), brandWrap);
    brandLabel->setObjectName(QStringLiteral("brandLabel"));
    auto* brandSubLabel = new QLabel(QStringLiteral("Mission Planning / Scenario Editing / Result Analysis"), brandWrap);
    brandSubLabel->setObjectName(QStringLiteral("brandSubLabel"));
    brandLayout->addWidget(brandLabel);
    brandLayout->addWidget(brandSubLabel);
    topBarLayout->addWidget(brandWrap);

    auto* navWrap = new QWidget(topBar);
    auto* navLayout = new QHBoxLayout(navWrap);
    navLayout->setContentsMargins(0, 0, 0, 0);
    navLayout->setSpacing(8);
    auto* planningNavButton = new QPushButton(QStringLiteral("任务规划"), navWrap);
    auto* scenarioNavButton = new QPushButton(QStringLiteral("场景编辑"), navWrap);
    auto* resultNavButton = new QPushButton(QStringLiteral("结果分析"), navWrap);
    planningNavButton->setObjectName(QStringLiteral("navButton"));
    scenarioNavButton->setObjectName(QStringLiteral("navButton"));
    resultNavButton->setObjectName(QStringLiteral("navButton"));
    planningNavButton->setCheckable(true);
    scenarioNavButton->setCheckable(true);
    resultNavButton->setCheckable(true);
    navLayout->addWidget(planningNavButton);
    navLayout->addWidget(scenarioNavButton);
    navLayout->addWidget(resultNavButton);
    topBarLayout->addWidget(navWrap);
    topBarLayout->addStretch(1);

    rootLayout->addWidget(topBar);

    auto* pageStack = new QStackedWidget(central);
    rootLayout->addWidget(pageStack, 1);

    auto* planningPage = new QWidget(pageStack);
    auto* planningLayout = new QHBoxLayout(planningPage);
    planningLayout->setContentsMargins(0, 0, 0, 0);
    planningLayout->setSpacing(8);

    auto* commandPanel = new QFrame(planningPage);
    commandPanel->setMinimumWidth(420);
    commandPanel->setMaximumWidth(460);
    auto* commandLayout = new QVBoxLayout(commandPanel);
    commandLayout->setContentsMargins(8, 8, 8, 8);
    commandLayout->setSpacing(8);

    auto* overviewGroup = new QGroupBox(QStringLiteral("任务态势摘要"), commandPanel);
    auto* overviewLayout = new QFormLayout(overviewGroup);
    m_blueForceCountValue = new QLabel(QStringLiteral("0"), overviewGroup);
    m_redTargetCountValue = new QLabel(QStringLiteral("0"), overviewGroup);
    m_threatZoneCountValue = new QLabel(QStringLiteral("0"), overviewGroup);
    m_scenarioBalanceValue = new QLabel(QStringLiteral("待配置"), overviewGroup);
    m_planHealthValue = new QLabel(QStringLiteral("未规划"), overviewGroup);
    overviewLayout->addRow(QStringLiteral("蓝方导弹"), m_blueForceCountValue);
    overviewLayout->addRow(QStringLiteral("红方目标"), m_redTargetCountValue);
    overviewLayout->addRow(QStringLiteral("威胁区"), m_threatZoneCountValue);
    overviewLayout->addRow(QStringLiteral("战场均衡度"), m_scenarioBalanceValue);
    overviewLayout->addRow(QStringLiteral("方案健康度"), m_planHealthValue);
    commandLayout->addWidget(overviewGroup);

    auto* algoGroup = new QGroupBox(QStringLiteral("分配算法与战术参数"), commandPanel);
    auto* algoLayout = new QFormLayout(algoGroup);

    m_allocationCombo = new QComboBox(algoGroup);
    m_allocationCombo->addItems({
        QStringLiteral("自动优选（四算法并行评估）"),
        QStringLiteral("匈牙利算法（最优分配）"),
        QStringLiteral("遗传算法（复杂场景）"),
        QStringLiteral("贪心算法（快速估算）"),
        QStringLiteral("优先级均衡贪心（改进）")});

    m_profileCombo = new QComboBox(algoGroup);
    m_profileCombo->addItems({
        QStringLiteral("低空隐蔽突防"),
        QStringLiteral("高空快速突防"),
        QStringLiteral("混合剖面突防")});

    m_clearanceSpin = createSpinBox(80.0, 2000.0, 320.0, 0, 20.0, algoGroup);
    m_gridStepSpin = createSpinBox(0.01, 0.2, 0.04, 3, 0.005, algoGroup);
    m_threatPenaltyCheck = new QCheckBox(QStringLiteral("启用威胁区软惩罚"), algoGroup);
    m_threatPenaltyCheck->setChecked(true);

    algoLayout->addRow(QStringLiteral("分配算法"), m_allocationCombo);
    algoLayout->addRow(QStringLiteral("突防剖面"), m_profileCombo);
    algoLayout->addRow(QStringLiteral("离地裕度(m)"), m_clearanceSpin);
    algoLayout->addRow(QStringLiteral("网格分辨率(°)"), m_gridStepSpin);
    algoLayout->addRow(QStringLiteral(""), m_threatPenaltyCheck);
    commandLayout->addWidget(algoGroup);

    auto* actionGroup = new QGroupBox(QStringLiteral("规划与推演"), commandPanel);
    auto* actionLayout = new QVBoxLayout(actionGroup);
    auto* planButton = new QPushButton(QStringLiteral("执行多导弹协同规划"), actionGroup);
    auto* simButton = new QPushButton(QStringLiteral("开始多弹三维推演"), actionGroup);

    auto* speedLayout = new QFormLayout;
    m_timeScaleSpin = createSpinBox(1.0, 240.0, 20.0, 0, 1.0, actionGroup);
    m_followMissileCheck = new QCheckBox(QStringLiteral("跟随导弹视角（可随时鼠标接管）"), actionGroup);
    m_followMissileCheck->setChecked(false);
    speedLayout->addRow(QStringLiteral("推演倍率(x)"), m_timeScaleSpin);

    actionLayout->addWidget(planButton);
    actionLayout->addWidget(simButton);
    actionLayout->addLayout(speedLayout);
    actionLayout->addWidget(m_followMissileCheck);
    commandLayout->addWidget(actionGroup);

    auto* replanGroup = new QGroupBox(QStringLiteral("动态重规划"), commandPanel);
    auto* replanLayout = new QHBoxLayout(replanGroup);
    m_failureMissileCombo = new QComboBox(replanGroup);
    auto* failButton = new QPushButton(QStringLiteral("模拟导弹失效"), replanGroup);
    auto* replanButton = new QPushButton(QStringLiteral("执行动态重规划"), replanGroup);
    replanLayout->addWidget(m_failureMissileCombo, 1);
    replanLayout->addWidget(failButton);
    replanLayout->addWidget(replanButton);
    commandLayout->addWidget(replanGroup);

    auto* mapToolsGroup = new QGroupBox(QStringLiteral("地图控制"), commandPanel);
    auto* mapToolsLayout = new QHBoxLayout(mapToolsGroup);
    auto* zoomOutButton = new QPushButton(QStringLiteral("- 缩小"), mapToolsGroup);
    auto* zoomInButton = new QPushButton(QStringLiteral("+ 放大"), mapToolsGroup);
    mapToolsLayout->addWidget(zoomOutButton);
    mapToolsLayout->addWidget(zoomInButton);
    commandLayout->addWidget(mapToolsGroup);
    commandLayout->addStretch(1);

    auto* planningRightPane = new QWidget(planningPage);
    auto* planningRightLayout = new QVBoxLayout(planningRightPane);
    planningRightLayout->setContentsMargins(0, 0, 0, 0);
    planningRightLayout->setSpacing(8);

    auto* telemetryGroup = new QGroupBox(
        QStringLiteral("飞行遥测分析（高度 / 弹目距 / 速度 / 俯仰 / 航向 / 加速度）"),
        planningRightPane);
    auto* telemetryLayout = new QVBoxLayout(telemetryGroup);
    telemetryLayout->setContentsMargins(8, 10, 8, 8);
    m_telemetryWidget = new TelemetryPlotWidget(telemetryGroup);
    telemetryLayout->addWidget(m_telemetryWidget);
    telemetryGroup->setMinimumHeight(320);

    auto* mapGroup = new QGroupBox(QStringLiteral("任务规划态势图（导弹 / 目标 / 威胁）"), planningRightPane);
    auto* mapLayout = new QVBoxLayout(mapGroup);
    mapLayout->setContentsMargins(6, 10, 6, 6);
    m_earthWidget = new OsgEarthWidget(mapGroup);
    mapLayout->addWidget(m_earthWidget);

    planningRightLayout->addWidget(telemetryGroup, 0);
    planningRightLayout->addWidget(mapGroup, 1);

    planningLayout->addWidget(commandPanel, 0);
    planningLayout->addWidget(planningRightPane, 1);

    auto* scenarioPage = new QWidget(pageStack);
    auto* scenarioLayout = new QVBoxLayout(scenarioPage);
    scenarioLayout->setContentsMargins(0, 0, 0, 0);
    scenarioLayout->setSpacing(8);

    auto* sceneGroup = new QGroupBox(QStringLiteral("场景模式与地球数据"), scenarioPage);
    auto* sceneLayout = new QFormLayout(sceneGroup);
    m_globeModeCombo = new QComboBox(sceneGroup);
    m_globeModeCombo->addItems({
        QStringLiteral("真实卫星地球（离线数据优先）"),
        QStringLiteral("演示态势地球（简化模型）")});
    m_sceneDataSourceValue = new QLabel(QStringLiteral("待检测（运行后加载）"), sceneGroup);
    auto* reloadPresetButton = new QPushButton(QStringLiteral("重载典型战场样例"), sceneGroup);
    sceneLayout->addRow(QStringLiteral("地球模型"), m_globeModeCombo);
    sceneLayout->addRow(QStringLiteral("数据源"), m_sceneDataSourceValue);
    sceneLayout->addRow(QStringLiteral("快速填充"), reloadPresetButton);
    scenarioLayout->addWidget(sceneGroup);

    auto* entitiesSplitter = new QSplitter(Qt::Horizontal, scenarioPage);
    entitiesSplitter->setChildrenCollapsible(false);
    entitiesSplitter->setHandleWidth(8);

    auto* missileGroup = new QGroupBox(QStringLiteral("蓝方导弹编组"), entitiesSplitter);
    auto* missileOuterLayout = new QVBoxLayout(missileGroup);

    auto* missileListRow = new QHBoxLayout;
    m_missileList = new QListWidget(missileGroup);
    m_missileList->setMinimumHeight(460);
    auto* missileAddBtn = new QPushButton(QStringLiteral("添加"), missileGroup);
    missileAddBtn->setMinimumWidth(72);
    auto* missileRemoveBtn = new QPushButton(QStringLiteral("移除"), missileGroup);
    missileRemoveBtn->setMinimumWidth(72);
    auto* missileBtnCol = new QVBoxLayout;
    missileBtnCol->addWidget(missileAddBtn);
    missileBtnCol->addWidget(missileRemoveBtn);
    missileBtnCol->addStretch();
    missileListRow->addWidget(m_missileList, 1);
    missileListRow->addLayout(missileBtnCol);
    missileOuterLayout->addLayout(missileListRow);

    auto* missileParamLayout = new QGridLayout;
    m_missileLon = createSpinBox(-180.0, 180.0, 103.80, 4, 0.01, missileGroup);
    m_missileLat = createSpinBox(-90.0, 90.0, 32.10, 4, 0.01, missileGroup);
    m_missileAlt = createSpinBox(0.0, 120000.0, 1800.0, 0, 100.0, missileGroup);
    m_missileTypeCombo = new QComboBox(missileGroup);
    m_missileTypeCombo->addItems({
        QStringLiteral("高亚音速"), QStringLiteral("超音速"), QStringLiteral("高超滑翔")});
    m_missileSpeedSpin = createSpinBox(30.0, 2500.0, 900.0, 0, 20.0, missileGroup);

    int r = 0;
    missileParamLayout->addWidget(new QLabel(QStringLiteral("经度"), missileGroup), r, 0);
    missileParamLayout->addWidget(m_missileLon, r, 1);
    missileParamLayout->addWidget(new QLabel(QStringLiteral("纬度"), missileGroup), r, 2);
    missileParamLayout->addWidget(m_missileLat, r, 3);
    ++r;
    missileParamLayout->addWidget(new QLabel(QStringLiteral("高度(m)"), missileGroup), r, 0);
    missileParamLayout->addWidget(m_missileAlt, r, 1);
    missileParamLayout->addWidget(new QLabel(QStringLiteral("型号"), missileGroup), r, 2);
    missileParamLayout->addWidget(m_missileTypeCombo, r, 3);
    ++r;
    missileParamLayout->addWidget(new QLabel(QStringLiteral("速度(m/s)"), missileGroup), r, 0);
    missileParamLayout->addWidget(m_missileSpeedSpin, r, 1);
    missileOuterLayout->addLayout(missileParamLayout);
    missileOuterLayout->addWidget(new QLabel(QStringLiteral("提示：双击条目后可直接修改参数并实时同步到地球态势图。"), missileGroup));

    auto* targetGroup = new QGroupBox(QStringLiteral("红方目标编组"), entitiesSplitter);
    auto* targetOuterLayout = new QVBoxLayout(targetGroup);

    auto* targetListRow = new QHBoxLayout;
    m_targetList = new QListWidget(targetGroup);
    m_targetList->setMinimumHeight(460);
    auto* targetAddBtn = new QPushButton(QStringLiteral("添加"), targetGroup);
    targetAddBtn->setMinimumWidth(72);
    auto* targetRemoveBtn = new QPushButton(QStringLiteral("移除"), targetGroup);
    targetRemoveBtn->setMinimumWidth(72);
    auto* targetBtnCol = new QVBoxLayout;
    targetBtnCol->addWidget(targetAddBtn);
    targetBtnCol->addWidget(targetRemoveBtn);
    targetBtnCol->addStretch();
    targetListRow->addWidget(m_targetList, 1);
    targetListRow->addLayout(targetBtnCol);
    targetOuterLayout->addLayout(targetListRow);

    auto* targetParamLayout = new QGridLayout;
    m_targetLon = createSpinBox(-180.0, 180.0, 118.40, 4, 0.01, targetGroup);
    m_targetLat = createSpinBox(-90.0, 90.0, 40.20, 4, 0.01, targetGroup);
    m_targetAlt = createSpinBox(0.0, 120000.0, 2200.0, 0, 100.0, targetGroup);
    m_targetPrioritySpin = new QSpinBox(targetGroup);
    m_targetPrioritySpin->setRange(1, 10);
    m_targetPrioritySpin->setValue(5);
    m_targetPrioritySpin->setToolTip(QStringLiteral("1=最低, 10=最高优先级"));

    r = 0;
    targetParamLayout->addWidget(new QLabel(QStringLiteral("经度"), targetGroup), r, 0);
    targetParamLayout->addWidget(m_targetLon, r, 1);
    targetParamLayout->addWidget(new QLabel(QStringLiteral("纬度"), targetGroup), r, 2);
    targetParamLayout->addWidget(m_targetLat, r, 3);
    ++r;
    targetParamLayout->addWidget(new QLabel(QStringLiteral("高度(m)"), targetGroup), r, 0);
    targetParamLayout->addWidget(m_targetAlt, r, 1);
    targetParamLayout->addWidget(new QLabel(QStringLiteral("优先级(1-10)"), targetGroup), r, 2);
    targetParamLayout->addWidget(m_targetPrioritySpin, r, 3);
    targetOuterLayout->addLayout(targetParamLayout);
    targetOuterLayout->addWidget(new QLabel(QStringLiteral("提示：高价值目标建议优先级设为 8-10。"), targetGroup));

    auto* threatGroup = new QGroupBox(QStringLiteral("威胁区配置"), entitiesSplitter);
    auto* threatLayout = new QGridLayout(threatGroup);

    m_threatLon = createSpinBox(-180.0, 180.0, 111.2000, 4, 0.01, threatGroup);
    m_threatLat = createSpinBox(-90.0, 90.0, 36.6000, 4, 0.01, threatGroup);
    m_threatRadius = createSpinBox(500.0, 240000.0, 48000.0, 0, 1000.0, threatGroup);
    m_threatMaxAlt = createSpinBox(100.0, 60000.0, 18000.0, 0, 200.0, threatGroup);

    r = 0;
    threatLayout->addWidget(new QLabel(QStringLiteral("中心经度"), threatGroup), r, 0);
    threatLayout->addWidget(m_threatLon, r, 1);
    threatLayout->addWidget(new QLabel(QStringLiteral("中心纬度"), threatGroup), r, 2);
    threatLayout->addWidget(m_threatLat, r, 3);
    ++r;
    threatLayout->addWidget(new QLabel(QStringLiteral("半径(m)"), threatGroup), r, 0);
    threatLayout->addWidget(m_threatRadius, r, 1);
    threatLayout->addWidget(new QLabel(QStringLiteral("高度上限(m)"), threatGroup), r, 2);
    threatLayout->addWidget(m_threatMaxAlt, r, 3);

    auto* addThreatButton = new QPushButton(QStringLiteral("添加威胁区"), threatGroup);
    auto* clearThreatButton = new QPushButton(QStringLiteral("清空威胁区"), threatGroup);
    ++r;
    threatLayout->addWidget(addThreatButton, r, 0, 1, 2);
    threatLayout->addWidget(clearThreatButton, r, 2, 1, 2);

    m_threatList = new QListWidget(threatGroup);
    m_threatList->setMinimumHeight(260);
    ++r;
    threatLayout->addWidget(m_threatList, r, 0, 1, 4);
    ++r;
    threatLayout->addWidget(new QLabel(QStringLiteral("建议：半径 8-50km、上限 8-20km 可覆盖常见防空拦截包线。"), threatGroup), r, 0, 1, 4);

    entitiesSplitter->addWidget(missileGroup);
    entitiesSplitter->addWidget(targetGroup);
    entitiesSplitter->addWidget(threatGroup);
    entitiesSplitter->setStretchFactor(0, 1);
    entitiesSplitter->setStretchFactor(1, 1);
    entitiesSplitter->setStretchFactor(2, 1);
    scenarioLayout->addWidget(entitiesSplitter, 1);

    auto* resultPage = new QWidget(pageStack);
    auto* resultLayout = new QHBoxLayout(resultPage);
    resultLayout->setContentsMargins(0, 0, 0, 0);
    resultLayout->setSpacing(8);

    auto* analysisPane = new QWidget(resultPage);
    auto* analysisLayout = new QVBoxLayout(analysisPane);
    analysisLayout->setContentsMargins(0, 0, 0, 0);
    analysisLayout->setSpacing(8);

    auto* assignGroup = new QGroupBox(QStringLiteral("分配结果详情"), analysisPane);
    auto* assignLayout = new QVBoxLayout(assignGroup);
    m_assignmentTable = new QTableWidget(assignGroup);
    m_assignmentTable->setColumnCount(4);
    m_assignmentTable->setHorizontalHeaderLabels({
        QStringLiteral("导弹"), QStringLiteral("目标"), QStringLiteral("优先级"), QStringLiteral("状态")});
    m_assignmentTable->horizontalHeader()->setStretchLastSection(true);
    m_assignmentTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_assignmentTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_assignmentTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    assignLayout->addWidget(m_assignmentTable);
    analysisLayout->addWidget(assignGroup, 1);

    auto* algoCompareGroup = new QGroupBox(QStringLiteral("算法对比分析（四算法）"), analysisPane);
    auto* algoCompareLayout = new QVBoxLayout(algoCompareGroup);
    m_algoCompareTable = new QTableWidget(algoCompareGroup);
    m_algoCompareTable->setColumnCount(7);
    m_algoCompareTable->setHorizontalHeaderLabels({
        QStringLiteral("算法"),
        QStringLiteral("是否采用"),
        QStringLiteral("成功率"),
        QStringLiteral("成功/失败"),
        QStringLiteral("规划耗时(ms)"),
        QStringLiteral("平均航程(km)"),
        QStringLiteral("综合评分")});
    m_algoCompareTable->horizontalHeader()->setStretchLastSection(true);
    m_algoCompareTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_algoCompareTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_algoCompareTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    algoCompareLayout->addWidget(m_algoCompareTable);
    analysisLayout->addWidget(algoCompareGroup, 1);

    auto* resultSidePanel = new QFrame(resultPage);
    resultSidePanel->setMinimumWidth(370);
    resultSidePanel->setMaximumWidth(430);
    auto* resultSideLayout = new QVBoxLayout(resultSidePanel);
    resultSideLayout->setContentsMargins(8, 8, 8, 8);
    resultSideLayout->setSpacing(8);

    auto* metricGroup = new QGroupBox(QStringLiteral("任务指标"), resultSidePanel);
    auto* metricLayout = new QFormLayout(metricGroup);

    m_planTimeValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_pathLengthValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_nodesValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_simProgressValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_etaValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_phaseValue = new QLabel(QStringLiteral("待命"), metricGroup);
    m_currentSpeedValue = new QLabel(QStringLiteral("0.0"), metricGroup);
    m_bestAlgoValue = new QLabel(QStringLiteral("--"), metricGroup);

    metricLayout->addRow(QStringLiteral("规划耗时(ms)"), m_planTimeValue);
    metricLayout->addRow(QStringLiteral("航程长度(m)"), m_pathLengthValue);
    metricLayout->addRow(QStringLiteral("扩展节点数"), m_nodesValue);
    metricLayout->addRow(QStringLiteral("推演进度"), m_simProgressValue);
    metricLayout->addRow(QStringLiteral("预计剩余(s)"), m_etaValue);
    metricLayout->addRow(QStringLiteral("飞行阶段"), m_phaseValue);
    metricLayout->addRow(QStringLiteral("当前速度(m/s)"), m_currentSpeedValue);
    metricLayout->addRow(QStringLiteral("当前规划算法"), m_bestAlgoValue);
    resultSideLayout->addWidget(metricGroup);

    auto* effectGroup = new QGroupBox(QStringLiteral("任务效果评估"), resultSidePanel);
    auto* effectLayout = new QFormLayout(effectGroup);
    m_successRateValue = new QLabel(QStringLiteral("--"), effectGroup);
    m_successCountValue = new QLabel(QStringLiteral("--"), effectGroup);
    m_failureCountValue = new QLabel(QStringLiteral("--"), effectGroup);
    m_totalTargetsValue = new QLabel(QStringLiteral("--"), effectGroup);
    effectLayout->addRow(QStringLiteral("总体成功率"), m_successRateValue);
    effectLayout->addRow(QStringLiteral("命中/失效"), m_successCountValue);
    effectLayout->addRow(QStringLiteral("失效导弹"), m_failureCountValue);
    effectLayout->addRow(QStringLiteral("目标总数"), m_totalTargetsValue);
    resultSideLayout->addWidget(effectGroup);

    auto* reportGroup = new QGroupBox(QStringLiteral("报告与复盘"), resultSidePanel);
    auto* reportLayout = new QVBoxLayout(reportGroup);
    m_exportHtmlButton = new QPushButton(QStringLiteral("导出 HTML 报告"), reportGroup);
    m_exportHtmlButton->setEnabled(false);
    reportLayout->addWidget(m_exportHtmlButton);
    reportLayout->addWidget(new QLabel(QStringLiteral("建议在推演结束后导出，便于归档与复盘。"), reportGroup));
    resultSideLayout->addWidget(reportGroup);
    resultSideLayout->addStretch(1);

    resultLayout->addWidget(analysisPane, 1);
    resultLayout->addWidget(resultSidePanel, 0);

    pageStack->addWidget(planningPage);
    pageStack->addWidget(scenarioPage);
    pageStack->addWidget(resultPage);
    pageStack->setCurrentIndex(0);

    auto switchPage = [pageStack, planningNavButton, scenarioNavButton, resultNavButton](int pageIndex) {
        pageStack->setCurrentIndex(pageIndex);
        planningNavButton->setChecked(pageIndex == 0);
        scenarioNavButton->setChecked(pageIndex == 1);
        resultNavButton->setChecked(pageIndex == 2);
    };

    switchPage(0);

    setCentralWidget(central);

    connect(addThreatButton, &QPushButton::clicked, this, &MainWindow::onAddThreat);
    connect(clearThreatButton, &QPushButton::clicked, this, &MainWindow::onClearThreats);
    connect(planButton, &QPushButton::clicked, this, &MainWindow::onPlanRoute);
    connect(simButton, &QPushButton::clicked, this, &MainWindow::onStartSimulation);
    connect(failButton, &QPushButton::clicked, this, &MainWindow::onSimulateFailure);
    connect(replanButton, &QPushButton::clicked, this, &MainWindow::onDynamicReplan);
    connect(m_exportHtmlButton, &QPushButton::clicked, this, &MainWindow::onExportHtmlReport);

    connect(missileAddBtn, &QPushButton::clicked, this, &MainWindow::onAddMissile);
    connect(missileRemoveBtn, &QPushButton::clicked, this, &MainWindow::onRemoveMissile);
    connect(m_missileList, &QListWidget::currentRowChanged, this, &MainWindow::onMissileSelectionChanged);

    connect(targetAddBtn, &QPushButton::clicked, this, &MainWindow::onAddTarget);
    connect(targetRemoveBtn, &QPushButton::clicked, this, &MainWindow::onRemoveTarget);
    connect(m_targetList, &QListWidget::currentRowChanged, this, &MainWindow::onTargetSelectionChanged);

    connect(planningNavButton, &QPushButton::clicked, this, [switchPage]() { switchPage(0); });
    connect(scenarioNavButton, &QPushButton::clicked, this, [switchPage]() { switchPage(1); });
    connect(resultNavButton, &QPushButton::clicked, this, [switchPage]() { switchPage(2); });

    connect(zoomInButton, &QPushButton::clicked, this, [this]() {
        if (m_earthWidget != nullptr) m_earthWidget->zoomIn();
    });
    connect(zoomOutButton, &QPushButton::clicked, this, [this]() {
        if (m_earthWidget != nullptr) m_earthWidget->zoomOut();
    });

    connect(m_globeModeCombo, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (m_earthWidget == nullptr) return;
        const auto mode = index == 0
                              ? OsgEarthWidget::GlobeMode::Realistic
                              : OsgEarthWidget::GlobeMode::Presentation;
        m_earthWidget->setGlobeMode(mode);
        refreshSceneDataSourceLabel();
    });

    connect(reloadPresetButton, &QPushButton::clicked, this, [this]() {
        stopAllSimulations();
        m_missileConfigs.clear();
        m_targetConfigs.clear();
        m_threatZones.clear();
        m_missileRuntimes.clear();
        m_algorithmComparisons.clear();
        m_lastMultiResult = {};
        m_nextMissileId = 1;
        m_nextTargetId = 1;

        populateDefaultScenario();
        refreshMissileList();
        refreshTargetList();
        refreshThreatList();
        syncEarthWidgetFromConfig();
        if (m_earthWidget != nullptr) {
            m_earthWidget->setThreatZones(m_threatZones);
            m_earthWidget->focusOnAllRoutes();
        }
        updateAlgorithmCompareTable();
        resetTelemetryPanel();
        updateAssignmentTable();
        updateOverallMetrics();
        statusBar()->showMessage(QStringLiteral("已重载典型战场样例。"), 3000);
    });

    connect(m_missileTypeCombo, &QComboBox::currentIndexChanged, this, [this](int index) {
        m_missileSpeedSpin->setValue(defaultSpeedForType(index));
    });

    updateScenarioSummary();
    refreshSceneDataSourceLabel();
    updateAlgorithmCompareTable();
}

osgEarth::GeoPoint MainWindow::makeGeoPoint(
    const QDoubleSpinBox* lonSpin,
    const QDoubleSpinBox* latSpin,
    const QDoubleSpinBox* altSpin) const {
    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (wgs84 == nullptr || lonSpin == nullptr || latSpin == nullptr || altSpin == nullptr) {
        return {};
    }

    return osgEarth::GeoPoint(
        wgs84,
        lonSpin->value(),
        latSpin->value(),
        altSpin->value(),
        osgEarth::ALTMODE_ABSOLUTE);
}

void MainWindow::refreshThreatList() {
    if (m_threatList == nullptr) return;

    m_threatList->clear();
    for (std::size_t i = 0; i < m_threatZones.size(); ++i) {
        const auto& t = m_threatZones[i];
        m_threatList->addItem(QStringLiteral("#%1 %2°E %3°N R%4m H%5m")
                                  .arg(static_cast<int>(i + 1))
                                  .arg(t.longitudeDeg, 0, 'f', 2)
                                  .arg(t.latitudeDeg, 0, 'f', 2)
                                  .arg(t.radiusMeters, 0, 'f', 0)
                                  .arg(t.maxAltitudeMeters, 0, 'f', 0));
    }

    updateScenarioSummary();
}

void MainWindow::refreshMissileList() {
    if (m_missileList == nullptr) return;

    m_missileList->blockSignals(true);
    m_missileList->clear();
    for (const auto& mc : m_missileConfigs) {
        m_missileList->addItem(QStringLiteral("%1  %2°E %3°N")
                                   .arg(QString::fromStdString(mc.name))
                                   .arg(mc.startLonDeg, 0, 'f', 2)
                                   .arg(mc.startLatDeg, 0, 'f', 2));
    }
    m_missileList->blockSignals(false);

    if (m_telemetryWidget != nullptr) {
        m_telemetryWidget->setMissileCount(static_cast<int>(m_missileConfigs.size()));
        std::vector<std::string> names;
        names.reserve(m_missileConfigs.size());
        for (const auto& mc : m_missileConfigs) {
            names.push_back(mc.name);
        }
        m_telemetryWidget->setMissileNames(names);
    }

    updateScenarioSummary();
}

void MainWindow::refreshTargetList() {
    if (m_targetList == nullptr) return;

    m_targetList->clear();
    for (const auto& tc : m_targetConfigs) {
        m_targetList->addItem(QStringLiteral("%1  %2°E %3°N  P:%4")
                                  .arg(QString::fromStdString(tc.name))
                                  .arg(tc.lonDeg, 0, 'f', 2)
                                  .arg(tc.latDeg, 0, 'f', 2)
                                  .arg(tc.priority));
    }

    updateScenarioSummary();
}

void MainWindow::refreshMetrics(const mission::PlanMetrics& metrics) {
    m_planTimeValue->setText(QStringLiteral("%1").arg(metrics.planningTimeMs, 0, 'f', 2));
    m_pathLengthValue->setText(QStringLiteral("%1").arg(metrics.pathLengthMeters, 0, 'f', 1));
    m_nodesValue->setText(QStringLiteral("%1 / %2")
                              .arg(metrics.expandedNodes)
                              .arg(metrics.visitedNodes));
    m_simProgressValue->setText(QStringLiteral("--"));
    m_etaValue->setText(QStringLiteral("--"));
    m_phaseValue->setText(QStringLiteral("待命"));
    m_currentSpeedValue->setText(QStringLiteral("0.0"));
    resetTelemetryPanel();
}

void MainWindow::refreshSceneDataSourceLabel() {
    if (m_sceneDataSourceValue == nullptr) return;

    if (m_earthWidget == nullptr) {
        m_sceneDataSourceValue->setText(QStringLiteral("地图控件未就绪"));
        return;
    }

    m_sceneDataSourceValue->setText(m_earthWidget->realEarthStatusText());
}

void MainWindow::resetTelemetryPanel() {
    if (m_telemetryWidget != nullptr) {
        m_telemetryWidget->clearAllHistory();
    }
}

void MainWindow::setExportEnabled(bool enabled) {
    if (m_exportHtmlButton != nullptr) {
        m_exportHtmlButton->setEnabled(enabled);
    }
}

void MainWindow::updateTelemetryPanel(
    int missileIndex,
    const osgEarth::GeoPoint& missilePoint,
    const mission::MissileSim::State& simState,
    double realDeltaSeconds) {
    if (m_telemetryWidget == nullptr || !missilePoint.isValid()) return;

    auto& rt = m_missileRuntimes[missileIndex];

    double heading = 0.0;
    double pitch = 0.0;
    if (rt.hasTelemetryPrevPoint && rt.prevTelemetryPoint.isValid()) {
        heading = headingDegrees(rt.prevTelemetryPoint, missilePoint);
        pitch = pitchDegrees(rt.prevTelemetryPoint, missilePoint);
    }

    const double safeDelta = std::max(0.001, realDeltaSeconds);
    const double accel = (simState.currentSpeedMetersPerSecond - rt.prevTelemetrySpeed) / safeDelta;
    const double remaining = std::max(0.0, simState.totalMeters - simState.traveledMeters);

    TelemetryPlotWidget::Sample sample;
    sample.timeSeconds = simState.elapsedSeconds;
    sample.speedMetersPerSecond = simState.currentSpeedMetersPerSecond;
    sample.altitudeMeters = missilePoint.z();
    sample.pitchDegrees = pitch;
    sample.headingDegrees = heading;
    sample.remainingMeters = remaining;
    sample.accelerationMetersPerSecond2 = rt.hasTelemetryPrevPoint ? accel : 0.0;
    m_telemetryWidget->pushSample(missileIndex, sample);
    rt.telemetryHistory.push_back(sample);

    rt.prevTelemetryPoint = missilePoint;
    rt.hasTelemetryPrevPoint = true;
    rt.prevTelemetrySpeed = simState.currentSpeedMetersPerSecond;
}

void MainWindow::onExportHtmlReport() {
    if (m_missileRuntimes.empty()) {
        QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("当前没有可导出的飞行数据。"));
        return;
    }

    mission::FlightReportData report;
    report.title = QStringLiteral("导弹飞行实时可视化报告");
    report.generatedAt = QDateTime::currentDateTime();
    report.missiles = m_missileConfigs;
    report.targets = m_targetConfigs;
    report.threats = m_threatZones;
    report.planningResult = m_lastMultiResult;

    for (std::size_t i = 0; i < m_missileRuntimes.size(); ++i) {
        const auto& rt = m_missileRuntimes[i];
        mission::FlightRouteReport routeReport;
        routeReport.missile = rt.config;
        routeReport.routeWaypoints = rt.route;
        routeReport.telemetry = rt.telemetryHistory;

        for (const auto& assign : m_lastMultiResult.assignments) {
            if (assign.missileIndex == static_cast<int>(i)) {
                routeReport.assignment = assign;
                if (assign.targetIndex >= 0 && assign.targetIndex < static_cast<int>(m_targetConfigs.size())) {
                    routeReport.target = m_targetConfigs[assign.targetIndex];
                }
                break;
            }
        }

        report.routes.push_back(routeReport);
    }

    for (const auto& item : m_algorithmComparisons) {
        mission::AlgorithmCompareReport algo;
        algo.name = QString::fromStdString(item.name);
        algo.selected = item.selected;
        algo.successRate = item.result.successRate;
        algo.successCount = item.result.successCount;
        algo.failureCount = item.result.failureCount;
        algo.planningTimeMs = item.result.totalPlanningTimeMs;
        algo.score = item.score;

        double pathMeters = 0.0;
        int plannedCount = 0;
        for (const auto& a : item.result.assignments) {
            if (!a.planned) {
                continue;
            }
            pathMeters += a.planResult.metrics.pathLengthMeters;
            ++plannedCount;
        }
        if (plannedCount > 0) {
            algo.averagePathKm = (pathMeters / static_cast<double>(plannedCount)) / 1000.0;
        }

        report.algorithmComparisons.push_back(algo);
    }

    const QString defaultName = QDir::current().filePath(
        QStringLiteral("flight_report_%1.html").arg(report.generatedAt.toString(QStringLiteral("yyyyMMdd_HHmmss"))));
    const QString fileName = QFileDialog::getSaveFileName(
        this,
        QStringLiteral("导出 HTML 报告"),
        defaultName,
        QStringLiteral("HTML Files (*.html)"));
    if (fileName.isEmpty()) {
        return;
    }

    const QString html = mission::buildFlightReportHtml(report);
    QSaveFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, QStringLiteral("导出失败"), QStringLiteral("无法创建导出文件。"));
        return;
    }

    if (file.write(html.toUtf8()) < 0 || !file.commit()) {
        QMessageBox::critical(this, QStringLiteral("导出失败"), QStringLiteral("HTML 内容写入失败。"));
        return;
    }

    statusBar()->showMessage(QStringLiteral("HTML 报告已导出：%1").arg(fileName), 5000);
}

void MainWindow::updateAssignmentTable() {
    if (m_assignmentTable == nullptr) return;

    m_assignmentTable->setRowCount(static_cast<int>(m_lastMultiResult.assignments.size()));

    for (std::size_t i = 0; i < m_lastMultiResult.assignments.size(); ++i) {
        const auto& assign = m_lastMultiResult.assignments[i];

        const QString missileName = assign.missileIndex < static_cast<int>(m_missileConfigs.size())
                                        ? QString::fromStdString(m_missileConfigs[assign.missileIndex].name)
                                        : QStringLiteral("--");

        const QString targetName = assign.targetIndex >= 0 && assign.targetIndex < static_cast<int>(m_targetConfigs.size())
                                       ? QString::fromStdString(m_targetConfigs[assign.targetIndex].name)
                                       : QStringLiteral("未分配");

        const QString priority = assign.targetIndex >= 0 && assign.targetIndex < static_cast<int>(m_targetConfigs.size())
                                     ? QString::number(m_targetConfigs[assign.targetIndex].priority)
                                     : "--";

        const QString status = assign.planned
                                   ? QStringLiteral("规划成功")
                                   : (assign.targetIndex < 0
                                          ? QStringLiteral("未分配")
                                          : QStringLiteral("规划失败"));

        auto* missileItem = new QTableWidgetItem(missileName);
        auto* targetItem = new QTableWidgetItem(targetName);
        auto* priorityItem = new QTableWidgetItem(priority);
        auto* statusItem = new QTableWidgetItem(status);

        if (assign.planned) {
            statusItem->setForeground(QColor(100, 255, 140));
        } else if (assign.targetIndex < 0) {
            statusItem->setForeground(QColor(180, 180, 180));
        } else {
            statusItem->setForeground(QColor(255, 100, 100));
        }

        m_assignmentTable->setItem(static_cast<int>(i), 0, missileItem);
        m_assignmentTable->setItem(static_cast<int>(i), 1, targetItem);
        m_assignmentTable->setItem(static_cast<int>(i), 2, priorityItem);
        m_assignmentTable->setItem(static_cast<int>(i), 3, statusItem);
    }
}

void MainWindow::updateScenarioSummary() {
    if (m_blueForceCountValue != nullptr) {
        m_blueForceCountValue->setText(QStringLiteral("%1 枚").arg(static_cast<int>(m_missileConfigs.size())));
    }
    if (m_redTargetCountValue != nullptr) {
        m_redTargetCountValue->setText(QStringLiteral("%1 个").arg(static_cast<int>(m_targetConfigs.size())));
    }
    if (m_threatZoneCountValue != nullptr) {
        m_threatZoneCountValue->setText(QStringLiteral("%1 个").arg(static_cast<int>(m_threatZones.size())));
    }

    if (m_scenarioBalanceValue == nullptr) {
        return;
    }

    if (m_missileConfigs.empty() && m_targetConfigs.empty()) {
        m_scenarioBalanceValue->setText(QStringLiteral("待录入蓝红双方实体"));
        return;
    }

    if (m_targetConfigs.empty()) {
        m_scenarioBalanceValue->setText(QStringLiteral("仅蓝方：请补充红方目标"));
        return;
    }

    const double forceRatio = static_cast<double>(m_missileConfigs.size()) /
                              static_cast<double>(std::max<std::size_t>(1, m_targetConfigs.size()));
    QString forceText;
    if (forceRatio < 0.8) {
        forceText = QStringLiteral("蓝方火力偏弱");
    } else if (forceRatio > 1.4) {
        forceText = QStringLiteral("蓝方火力充足");
    } else {
        forceText = QStringLiteral("蓝红配比均衡");
    }

    const double threatDensity = static_cast<double>(m_threatZones.size()) /
                                 static_cast<double>(std::max<std::size_t>(1, m_targetConfigs.size()));
    QString threatText;
    if (threatDensity > 0.8) {
        threatText = QStringLiteral("威胁密度高");
    } else if (threatDensity > 0.3) {
        threatText = QStringLiteral("威胁密度中");
    } else {
        threatText = QStringLiteral("威胁密度低");
    }

    m_scenarioBalanceValue->setText(QStringLiteral("%1 | %2").arg(forceText, threatText));
}

void MainWindow::updateOverallMetrics() {
    int completed = 0;
    int failed = 0;
    int totalTargets = static_cast<int>(m_targetConfigs.size());

    for (const auto& rt : m_missileRuntimes) {
        if (rt.completed) ++completed;
        if (rt.failed) ++failed;
    }

    const double successRate = totalTargets > 0
                                   ? static_cast<double>(completed) / totalTargets
                                   : 0.0;

    if (m_successRateValue != nullptr) {
        m_successRateValue->setText(QStringLiteral("%1%").arg(successRate * 100.0, 0, 'f', 1));
    }
    if (m_successCountValue != nullptr) {
        m_successCountValue->setText(QStringLiteral("%1 / %2").arg(completed).arg(failed));
    }
    if (m_failureCountValue != nullptr) {
        m_failureCountValue->setText(QString::number(failed));
    }
    if (m_totalTargetsValue != nullptr) {
        m_totalTargetsValue->setText(QString::number(totalTargets));
    }

    if (m_planHealthValue != nullptr) {
        if (m_lastMultiResult.assignments.empty()) {
            m_planHealthValue->setText(QStringLiteral("未规划"));
        } else if (m_simulationTimer.isActive()) {
            m_planHealthValue->setText(QStringLiteral("推演中"));
        } else if (successRate >= 0.8) {
            m_planHealthValue->setText(QStringLiteral("优秀"));
        } else if (successRate >= 0.5) {
            m_planHealthValue->setText(QStringLiteral("可接受"));
        } else {
            m_planHealthValue->setText(QStringLiteral("需优化"));
        }
    }
}

void MainWindow::saveCurrentMissileParams() {
    if (m_selectedMissileIndex < 0 || m_selectedMissileIndex >= static_cast<int>(m_missileConfigs.size())) {
        return;
    }

    auto& cfg = m_missileConfigs[m_selectedMissileIndex];
    cfg.startLonDeg = m_missileLon->value();
    cfg.startLatDeg = m_missileLat->value();
    cfg.startAltMeters = m_missileAlt->value();
    cfg.missileType = m_missileTypeCombo->currentIndex();
    cfg.speedMps = m_missileSpeedSpin->value();

    syncEarthWidgetFromConfig();
}

void MainWindow::saveCurrentTargetParams() {
    if (m_selectedTargetIndex < 0 || m_selectedTargetIndex >= static_cast<int>(m_targetConfigs.size())) {
        return;
    }

    auto& cfg = m_targetConfigs[m_selectedTargetIndex];
    cfg.lonDeg = m_targetLon->value();
    cfg.latDeg = m_targetLat->value();
    cfg.altMeters = m_targetAlt->value();
    cfg.priority = m_targetPrioritySpin->value();

    syncEarthWidgetFromConfig();
}

void MainWindow::loadMissileParams(int index) {
    if (index < 0 || index >= static_cast<int>(m_missileConfigs.size())) return;

    const auto& cfg = m_missileConfigs[index];
    m_missileLon->blockSignals(true);
    m_missileLat->blockSignals(true);
    m_missileAlt->blockSignals(true);
    m_missileTypeCombo->blockSignals(true);
    m_missileSpeedSpin->blockSignals(true);

    m_missileLon->setValue(cfg.startLonDeg);
    m_missileLat->setValue(cfg.startLatDeg);
    m_missileAlt->setValue(cfg.startAltMeters);
    m_missileTypeCombo->setCurrentIndex(cfg.missileType);
    m_missileSpeedSpin->setValue(cfg.speedMps);

    m_missileLon->blockSignals(false);
    m_missileLat->blockSignals(false);
    m_missileAlt->blockSignals(false);
    m_missileTypeCombo->blockSignals(false);
    m_missileSpeedSpin->blockSignals(false);
}

void MainWindow::loadTargetParams(int index) {
    if (index < 0 || index >= static_cast<int>(m_targetConfigs.size())) return;

    const auto& cfg = m_targetConfigs[index];
    m_targetLon->blockSignals(true);
    m_targetLat->blockSignals(true);
    m_targetAlt->blockSignals(true);
    m_targetPrioritySpin->blockSignals(true);

    m_targetLon->setValue(cfg.lonDeg);
    m_targetLat->setValue(cfg.latDeg);
    m_targetAlt->setValue(cfg.altMeters);
    m_targetPrioritySpin->setValue(cfg.priority);

    m_targetLon->blockSignals(false);
    m_targetLat->blockSignals(false);
    m_targetAlt->blockSignals(false);
    m_targetPrioritySpin->blockSignals(false);
}

void MainWindow::syncEarthWidgetFromConfig() {
    if (m_earthWidget == nullptr) return;

    const auto* wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (wgs84 == nullptr) return;

    m_earthWidget->setMissileCount(static_cast<int>(m_missileConfigs.size()));

    for (std::size_t i = 0; i < m_missileConfigs.size(); ++i) {
        const auto& mc = m_missileConfigs[i];
        osgEarth::GeoPoint startPoint(
            wgs84, mc.startLonDeg, mc.startLatDeg, mc.startAltMeters, osgEarth::ALTMODE_ABSOLUTE);
        m_earthWidget->setMissileStartPoint(static_cast<int>(i), startPoint);
    }

    for (std::size_t j = 0; j < m_targetConfigs.size() && j < m_missileConfigs.size(); ++j) {
        const auto& tc = m_targetConfigs[j];
        osgEarth::GeoPoint targetPoint(
            wgs84, tc.lonDeg, tc.latDeg, tc.altMeters, osgEarth::ALTMODE_ABSOLUTE);
        m_earthWidget->setMissileTargetPoint(static_cast<int>(j), targetPoint);
    }
}

void MainWindow::populateDefaultScenario() {
    if (!m_missileConfigs.empty() || !m_targetConfigs.empty() || !m_threatZones.empty()) {
        return;
    }

    m_missileConfigs = {
        {"M1", "导弹-1", 103.80, 32.10, 1800.0, 0, 880.0},
        {"M2", "导弹-2", 105.20, 30.60, 2000.0, 1, 980.0},
        {"M3", "导弹-3", 107.00, 31.40, 2200.0, 1, 1020.0},
        {"M4", "导弹-4", 101.70, 29.90, 1600.0, 0, 860.0},
        {"M5", "导弹-5", 109.30, 28.70, 2600.0, 2, 1320.0},
        {"M6", "导弹-6", 111.40, 30.20, 2400.0, 2, 1280.0},
    };

    m_targetConfigs = {
        {"T1", "指挥中心A", 118.40, 40.20, 2200.0, 10},
        {"T2", "指挥中心B", 117.30, 39.40, 2100.0, 10},
        {"T3", "雷达站A", 116.70, 37.20, 1800.0, 8},
        {"T4", "补给站A", 114.90, 36.10, 1500.0, 6},
        {"T5", "防空阵地A", 115.60, 38.30, 1600.0, 7},
        {"T6", "通信节点", 119.20, 36.70, 1200.0, 5},
    };

    m_threatZones = {
        {111.20, 36.60, 48000.0, 0.0, 18000.0},
        {114.80, 37.80, 36000.0, 0.0, 16000.0},
        {116.10, 39.20, 42000.0, 0.0, 20000.0},
        {118.20, 35.90, 28000.0, 0.0, 12000.0},
        {113.40, 34.70, 25000.0, 0.0, 10000.0},
    };

    m_nextMissileId = static_cast<int>(m_missileConfigs.size()) + 1;
    m_nextTargetId = static_cast<int>(m_targetConfigs.size()) + 1;
}

void MainWindow::updateAlgorithmCompareTable() {
    if (m_algoCompareTable == nullptr) {
        return;
    }

    m_algoCompareTable->setRowCount(static_cast<int>(m_algorithmComparisons.size()));

    for (int i = 0; i < static_cast<int>(m_algorithmComparisons.size()); ++i) {
        const auto& item = m_algorithmComparisons[i];
        double pathMeters = 0.0;
        int plannedCount = 0;
        for (const auto& a : item.result.assignments) {
            if (!a.planned) {
                continue;
            }
            pathMeters += a.planResult.metrics.pathLengthMeters;
            ++plannedCount;
        }
        const double avgPathKm = plannedCount > 0
                                     ? (pathMeters / static_cast<double>(plannedCount)) / 1000.0
                                     : 0.0;

        auto* algoItem = new QTableWidgetItem(QString::fromStdString(item.name));
        auto* selectedItem = new QTableWidgetItem(item.selected ? QStringLiteral("是") : QStringLiteral("否"));
        auto* rateItem = new QTableWidgetItem(QStringLiteral("%1%").arg(item.result.successRate * 100.0, 0, 'f', 1));
        auto* hitItem = new QTableWidgetItem(QStringLiteral("%1 / %2").arg(item.result.successCount).arg(item.result.failureCount));
        auto* timeItem = new QTableWidgetItem(QStringLiteral("%1").arg(item.result.totalPlanningTimeMs, 0, 'f', 2));
        auto* pathItem = new QTableWidgetItem(QStringLiteral("%1").arg(avgPathKm, 0, 'f', 2));
        auto* scoreItem = new QTableWidgetItem(QStringLiteral("%1").arg(item.score, 0, 'f', 1));

        if (item.selected) {
            algoItem->setForeground(QColor(110, 241, 159));
            selectedItem->setForeground(QColor(110, 241, 159));
        }

        m_algoCompareTable->setItem(i, 0, algoItem);
        m_algoCompareTable->setItem(i, 1, selectedItem);
        m_algoCompareTable->setItem(i, 2, rateItem);
        m_algoCompareTable->setItem(i, 3, hitItem);
        m_algoCompareTable->setItem(i, 4, timeItem);
        m_algoCompareTable->setItem(i, 5, pathItem);
        m_algoCompareTable->setItem(i, 6, scoreItem);
    }

    if (m_bestAlgoValue != nullptr) {
        m_bestAlgoValue->setText(allocationMethodName(m_lastPlanningMethod));
    }
}

void MainWindow::stopAllSimulations() {
    m_simulationTimer.stop();
    for (auto& rt : m_missileRuntimes) {
        rt.active = false;
    }
}
