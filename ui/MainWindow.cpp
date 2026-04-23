#include "ui/MainWindow.h"

#include <algorithm>
#include <cmath>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

#include <osgEarth/SpatialReference>

#include "render/OsgEarthWidget.h"
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

}  // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    buildUi();

    connect(&m_simulationTimer, &QTimer::timeout, this, &MainWindow::onSimulationTick);
    m_simulationTimer.setInterval(16);

    m_nextMissileId = 1;
    m_nextTargetId = 1;

    refreshMissileList();
    refreshTargetList();
    syncEarthWidgetFromConfig();

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

    const int allocIndex = m_allocationCombo->currentIndex();
    switch (allocIndex) {
        case 0: options.method = mission::AllocationMethod::Hungarian; break;
        case 1: options.method = mission::AllocationMethod::Genetic; break;
        case 2: options.method = mission::AllocationMethod::Greedy; break;
        default: options.method = mission::AllocationMethod::Hungarian; break;
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

    m_lastMultiResult = planner.plan(m_missileConfigs, m_targetConfigs, m_threatZones, options);

    m_missileRuntimes.clear();
    m_missileRuntimes.resize(m_missileConfigs.size());
    for (std::size_t i = 0; i < m_missileConfigs.size(); ++i) {
        m_missileRuntimes[i].config = m_missileConfigs[i];
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

    if (!m_lastMultiResult.assignments.empty() && m_lastMultiResult.assignments.front().planned) {
        refreshMetrics(m_lastMultiResult.assignments.front().planResult.metrics);
    }

    refreshSceneDataSourceLabel();

    const QString methodNames[] = {QStringLiteral("匈牙利算法"), QStringLiteral("遗传算法"), QStringLiteral("贪心算法")};
    statusBar()->showMessage(
        QStringLiteral("多导弹规划完成(%1)：成功 %2 / 失败 %3，成功率 %4%")
            .arg(methodNames[allocIndex])
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
        m_earthWidget->focusOnAllRoutes();
    }

    m_tickClock.restart();
    m_lastTickMs = 0;

    resetTelemetryPanel();

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
        if (!ok || !rt.sim.isRunning()) {
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

        updateTelemetryPanel(static_cast<int>(i), missilePoint, rt.sim.state(), realDeltaSeconds);

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
    switch (allocIndex) {
        case 0: options.method = mission::AllocationMethod::Hungarian; break;
        case 1: options.method = mission::AllocationMethod::Genetic; break;
        case 2: options.method = mission::AllocationMethod::Greedy; break;
        default: options.method = mission::AllocationMethod::Hungarian; break;
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
}

void MainWindow::buildUi() {
    setWindowTitle(QStringLiteral("空面导弹多弹协同任务规划与验证可视化系统"));
    resize(1560, 920);

    setStyleSheet(QStringLiteral(
        "QMainWindow, QWidget { background: #0d1724; color: #d8e4f2; }"
        "QTabWidget::pane { border: 1px solid #28425d; top: -1px; background: #111e2e; }"
        "QTabBar::tab { background: #13263a; border: 1px solid #2f4f6f; padding: 6px 12px; margin-right: 2px; color: #9ec2e4; }"
        "QTabBar::tab:selected { background: #1a3550; color: #e7f4ff; }"
        "QGroupBox { border: 1px solid #2a3f58; border-radius: 4px; margin-top: 8px; padding-top: 6px; font-weight: 600; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 8px; color: #8fd0ff; }"
        "QFrame { background: #121f2f; border: 1px solid #2a3f58; border-radius: 4px; }"
        "QLabel { color: #d8e4f2; }"
        "QPushButton { background: #1f8ecd; border: 1px solid #3aa4e2; border-radius: 4px; padding: 6px 8px; color: #f2f9ff; font-weight: 600; }"
        "QPushButton:hover { background: #26a3eb; }"
        "QPushButton:pressed { background: #1779b2; }"
        "QDoubleSpinBox, QSpinBox, QComboBox, QListWidget, QTableWidget { background: #0f1b28; border: 1px solid #35506d; border-radius: 3px; color: #e8f3ff; selection-background-color: #2f77aa; }"
        "QTableWidget::item { padding: 2px; }"
        "QHeaderView::section { background: #13263a; color: #9ec2e4; border: 1px solid #2a3f58; padding: 3px; }"
        "QStatusBar { background: #0a131f; color: #9ec2e4; }"));

    auto* central = new QWidget(this);
    auto* rootLayout = new QHBoxLayout(central);
    rootLayout->setContentsMargins(8, 8, 8, 8);
    rootLayout->setSpacing(8);

    auto* panel = new QFrame(central);
    panel->setFrameShape(QFrame::StyledPanel);
    panel->setMinimumWidth(400);
    panel->setMaximumWidth(460);

    auto* panelLayout = new QVBoxLayout(panel);
    panelLayout->setContentsMargins(8, 8, 8, 8);
    panelLayout->setSpacing(6);

    auto* titleLabel = new QLabel(QStringLiteral("多弹协同任务控制台"), panel);
    QFont titleFont = titleLabel->font();
    titleFont.setPointSize(13);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);
    panelLayout->addWidget(titleLabel);

    auto* sceneGroup = new QGroupBox(QStringLiteral("场景模式"), panel);
    auto* sceneLayout = new QFormLayout(sceneGroup);
    m_globeModeCombo = new QComboBox(sceneGroup);
    m_globeModeCombo->addItems({
        QStringLiteral("真实卫星地球（离线数据优先）"),
        QStringLiteral("演示态势地球（简化模型）")});
    m_sceneDataSourceValue = new QLabel(QStringLiteral("待检测（运行后加载）"), sceneGroup);
    auto* offlineHint = new QLabel(
        QStringLiteral("离线数据推荐路径:\n"
                       "data/earth/highres_global.earth\n"
                       "可选: data/earth/world.earth / resources/world.earth / default.earth"),
        sceneGroup);
    offlineHint->setWordWrap(true);
    offlineHint->setStyleSheet(QStringLiteral("color: #8fb8dd;"));
    offlineHint->setMaximumHeight(48);
    sceneLayout->addRow(QStringLiteral("地球模型"), m_globeModeCombo);
    sceneLayout->addRow(QStringLiteral("数据源"), m_sceneDataSourceValue);
    sceneLayout->addRow(QStringLiteral("离线放置"), offlineHint);
    panelLayout->addWidget(sceneGroup);

    auto* tabs = new QTabWidget(panel);
    tabs->setDocumentMode(true);

    auto* missionTab = new QWidget(tabs);
    auto* missionLayout = new QVBoxLayout(missionTab);
    missionLayout->setContentsMargins(6, 6, 6, 6);
    missionLayout->setSpacing(6);

    auto* missileGroup = new QGroupBox(QStringLiteral("导弹配置"), missionTab);
    auto* missileOuterLayout = new QVBoxLayout(missileGroup);

    auto* missileListRow = new QHBoxLayout;
    m_missileList = new QListWidget(missileGroup);
    m_missileList->setMaximumHeight(80);
    auto* missileAddBtn = new QPushButton(QStringLiteral("+"), missileGroup);
    missileAddBtn->setMinimumWidth(50);
    auto* missileRemoveBtn = new QPushButton(QStringLiteral("-"), missileGroup);
    missileRemoveBtn->setMinimumWidth(50);
    auto* missileBtnCol = new QVBoxLayout;
    missileBtnCol->addWidget(missileAddBtn);
    missileBtnCol->addWidget(missileRemoveBtn);
    missileBtnCol->addStretch();
    missileListRow->addWidget(m_missileList, 1);
    missileListRow->addLayout(missileBtnCol);
    missileOuterLayout->addLayout(missileListRow);

    auto* missileParamLayout = new QGridLayout;
    m_missileLon = createSpinBox(-180.0, 180.0, 112.35, 4, 0.01, missileGroup);
    m_missileLat = createSpinBox(-90.0, 90.0, 34.70, 4, 0.01, missileGroup);
    m_missileAlt = createSpinBox(0.0, 30000.0, 1200.0, 0, 50.0, missileGroup);
    m_missileTypeCombo = new QComboBox(missileGroup);
    m_missileTypeCombo->addItems({
        QStringLiteral("高亚音速"), QStringLiteral("超音速"), QStringLiteral("高超滑翔")});
    m_missileSpeedSpin = createSpinBox(30.0, 1200.0, 250.0, 0, 10.0, missileGroup);

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

    missionLayout->addWidget(missileGroup);

    auto* targetGroup = new QGroupBox(QStringLiteral("目标配置"), missionTab);
    auto* targetOuterLayout = new QVBoxLayout(targetGroup);

    auto* targetListRow = new QHBoxLayout;
    m_targetList = new QListWidget(targetGroup);
    m_targetList->setMaximumHeight(80);
    auto* targetAddBtn = new QPushButton(QStringLiteral("+"), targetGroup);
    targetAddBtn->setMinimumWidth(50);
    auto* targetRemoveBtn = new QPushButton(QStringLiteral("-"), targetGroup);
    targetRemoveBtn->setMinimumWidth(50);
    auto* targetBtnCol = new QVBoxLayout;
    targetBtnCol->addWidget(targetAddBtn);
    targetBtnCol->addWidget(targetRemoveBtn);
    targetBtnCol->addStretch();
    targetListRow->addWidget(m_targetList, 1);
    targetListRow->addLayout(targetBtnCol);
    targetOuterLayout->addLayout(targetListRow);

    auto* targetParamLayout = new QGridLayout;
    m_targetLon = createSpinBox(-180.0, 180.0, 113.30, 4, 0.01, targetGroup);
    m_targetLat = createSpinBox(-90.0, 90.0, 35.15, 4, 0.01, targetGroup);
    m_targetAlt = createSpinBox(0.0, 30000.0, 1600.0, 0, 50.0, targetGroup);
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

    missionLayout->addWidget(targetGroup);

    auto* threatGroup = new QGroupBox(QStringLiteral("雷达威胁区"), missionTab);
    auto* threatLayout = new QGridLayout(threatGroup);

    m_threatLon = createSpinBox(-180.0, 180.0, 112.8000, 4, 0.01, threatGroup);
    m_threatLat = createSpinBox(-90.0, 90.0, 34.9300, 4, 0.01, threatGroup);
    m_threatRadius = createSpinBox(500.0, 120000.0, 22000.0, 0, 500.0, threatGroup);
    m_threatMaxAlt = createSpinBox(100.0, 20000.0, 3000.0, 0, 100.0, threatGroup);

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
    m_threatList->setMaximumHeight(60);
    ++r;
    threatLayout->addWidget(m_threatList, r, 0, 1, 4);

    missionLayout->addWidget(threatGroup);

    auto* algoGroup = new QGroupBox(QStringLiteral("分配算法与战术参数"), missionTab);
    auto* algoLayout = new QFormLayout(algoGroup);

    m_allocationCombo = new QComboBox(algoGroup);
    m_allocationCombo->addItems({
        QStringLiteral("匈牙利算法（最优分配）"),
        QStringLiteral("遗传算法（复杂场景）"),
        QStringLiteral("贪心算法（快速估算）")});

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

    missionLayout->addWidget(algoGroup);
    missionLayout->addStretch(1);

    auto* executeTab = new QWidget(tabs);
    auto* executeLayout = new QVBoxLayout(executeTab);
    executeLayout->setContentsMargins(6, 6, 6, 6);
    executeLayout->setSpacing(6);

    auto* actionGroup = new QGroupBox(QStringLiteral("规划与推演"), executeTab);
    auto* actionLayout = new QVBoxLayout(actionGroup);

    auto* planButton = new QPushButton(QStringLiteral("执行多导弹协同规划"), actionGroup);
    auto* simButton = new QPushButton(QStringLiteral("开始多弹三维推演"), actionGroup);

    auto* speedLayout = new QFormLayout;
    m_timeScaleSpin = createSpinBox(1.0, 240.0, 45.0, 0, 1.0, actionGroup);
    m_followMissileCheck = new QCheckBox(QStringLiteral("跟随导弹视角（可随时鼠标接管）"), actionGroup);
    m_followMissileCheck->setChecked(false);

    speedLayout->addRow(QStringLiteral("推演倍率(x)"), m_timeScaleSpin);

    actionLayout->addWidget(planButton);
    actionLayout->addWidget(simButton);
    actionLayout->addLayout(speedLayout);
    actionLayout->addWidget(m_followMissileCheck);

    executeLayout->addWidget(actionGroup);

    auto* assignGroup = new QGroupBox(QStringLiteral("分配结果"), executeTab);
    auto* assignLayout = new QVBoxLayout(assignGroup);

    m_assignmentTable = new QTableWidget(assignGroup);
    m_assignmentTable->setColumnCount(4);
    m_assignmentTable->setHorizontalHeaderLabels({
        QStringLiteral("导弹"), QStringLiteral("目标"), QStringLiteral("优先级"), QStringLiteral("状态")});
    m_assignmentTable->horizontalHeader()->setStretchLastSection(true);
    m_assignmentTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_assignmentTable->setMaximumHeight(220);
    m_assignmentTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_assignmentTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    assignLayout->addWidget(m_assignmentTable);

    executeLayout->addWidget(assignGroup);

    auto* metricGroup = new QGroupBox(QStringLiteral("任务指标"), executeTab);
    auto* metricLayout = new QFormLayout(metricGroup);

    m_planTimeValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_pathLengthValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_nodesValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_simProgressValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_etaValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_phaseValue = new QLabel(QStringLiteral("待命"), metricGroup);
    m_currentSpeedValue = new QLabel(QStringLiteral("0.0"), metricGroup);
    m_successRateValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_successCountValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_failureCountValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_totalTargetsValue = new QLabel(QStringLiteral("--"), metricGroup);

    metricLayout->addRow(QStringLiteral("规划耗时(ms)"), m_planTimeValue);
    metricLayout->addRow(QStringLiteral("航程长度(m)"), m_pathLengthValue);
    metricLayout->addRow(QStringLiteral("扩展节点数"), m_nodesValue);
    metricLayout->addRow(QStringLiteral("推演进度"), m_simProgressValue);
    metricLayout->addRow(QStringLiteral("预计剩余(s)"), m_etaValue);
    metricLayout->addRow(QStringLiteral("飞行阶段"), m_phaseValue);
    metricLayout->addRow(QStringLiteral("当前速度(m/s)"), m_currentSpeedValue);
    metricLayout->addRow(QStringLiteral("成功率"), m_successRateValue);
    metricLayout->addRow(QStringLiteral("命中/失效"), m_successCountValue);
    metricLayout->addRow(QStringLiteral("目标总数"), m_totalTargetsValue);

    executeLayout->addWidget(metricGroup);

    auto* replanGroup = new QGroupBox(QStringLiteral("动态重规划"), executeTab);
    auto* replanLayout = new QHBoxLayout(replanGroup);

    m_failureMissileCombo = new QComboBox(replanGroup);
    auto* failButton = new QPushButton(QStringLiteral("模拟导弹失效"), replanGroup);
    auto* replanButton = new QPushButton(QStringLiteral("执行动态重规划"), replanGroup);

    replanLayout->addWidget(m_failureMissileCombo, 1);
    replanLayout->addWidget(failButton);
    replanLayout->addWidget(replanButton);

    executeLayout->addWidget(replanGroup);

    auto* zoomButtonsLayout = new QHBoxLayout;
    auto* zoomOutButton = new QPushButton(QStringLiteral("- 缩小"), executeTab);
    auto* zoomInButton = new QPushButton(QStringLiteral("+ 放大"), executeTab);
    zoomButtonsLayout->addWidget(zoomOutButton);
    zoomButtonsLayout->addWidget(zoomInButton);

    executeLayout->addLayout(zoomButtonsLayout);
    executeLayout->addStretch(1);

    tabs->addTab(missionTab, QStringLiteral("任务设定"));
    tabs->addTab(executeTab, QStringLiteral("推演监控"));
    panelLayout->addWidget(tabs, 1);

    auto* rightPane = new QWidget(central);
    auto* rightLayout = new QVBoxLayout(rightPane);
    rightLayout->setContentsMargins(0, 0, 0, 0);
    rightLayout->setSpacing(8);

    auto* telemetryGroup = new QGroupBox(
        QStringLiteral("飞行实时可视化（多弹遥测：高度/弹目距/速度/俯仰/航向/加速度）"),
        rightPane);
    auto* telemetryLayout = new QVBoxLayout(telemetryGroup);
    telemetryLayout->setContentsMargins(8, 10, 8, 8);

    m_telemetryWidget = new TelemetryPlotWidget(telemetryGroup);
    telemetryLayout->addWidget(m_telemetryWidget);
    telemetryGroup->setMinimumHeight(360);
    telemetryGroup->setMaximumHeight(460);

    auto* globeGroup = new QGroupBox(QStringLiteral("三维地球态势"), rightPane);
    auto* globeLayout = new QVBoxLayout(globeGroup);
    globeLayout->setContentsMargins(6, 10, 6, 6);
    m_earthWidget = new OsgEarthWidget(globeGroup);
    globeLayout->addWidget(m_earthWidget);

    rightLayout->addWidget(telemetryGroup, 0);
    rightLayout->addWidget(globeGroup, 1);

    rootLayout->addWidget(panel, 0);
    rootLayout->addWidget(rightPane, 1);

    setCentralWidget(central);

    connect(addThreatButton, &QPushButton::clicked, this, &MainWindow::onAddThreat);
    connect(clearThreatButton, &QPushButton::clicked, this, &MainWindow::onClearThreats);
    connect(planButton, &QPushButton::clicked, this, &MainWindow::onPlanRoute);
    connect(simButton, &QPushButton::clicked, this, &MainWindow::onStartSimulation);
    connect(failButton, &QPushButton::clicked, this, &MainWindow::onSimulateFailure);
    connect(replanButton, &QPushButton::clicked, this, &MainWindow::onDynamicReplan);

    connect(missileAddBtn, &QPushButton::clicked, this, &MainWindow::onAddMissile);
    connect(missileRemoveBtn, &QPushButton::clicked, this, &MainWindow::onRemoveMissile);
    connect(m_missileList, &QListWidget::currentRowChanged, this, &MainWindow::onMissileSelectionChanged);

    connect(targetAddBtn, &QPushButton::clicked, this, &MainWindow::onAddTarget);
    connect(targetRemoveBtn, &QPushButton::clicked, this, &MainWindow::onRemoveTarget);
    connect(m_targetList, &QListWidget::currentRowChanged, this, &MainWindow::onTargetSelectionChanged);

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

    connect(m_missileTypeCombo, &QComboBox::currentIndexChanged, this, [this](int index) {
        m_missileSpeedSpin->setValue(defaultSpeedForType(index));
    });

    refreshSceneDataSourceLabel();
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

    rt.prevTelemetryPoint = missilePoint;
    rt.hasTelemetryPrevPoint = true;
    rt.prevTelemetrySpeed = simState.currentSpeedMetersPerSecond;
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

    m_successRateValue->setText(QStringLiteral("%1%").arg(successRate * 100.0, 0, 'f', 1));
    m_successCountValue->setText(QStringLiteral("%1 / %2").arg(completed).arg(failed));
    m_totalTargetsValue->setText(QString::number(totalTargets));
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

void MainWindow::stopAllSimulations() {
    m_simulationTimer.stop();
    for (auto& rt : m_missileRuntimes) {
        rt.active = false;
    }
}
