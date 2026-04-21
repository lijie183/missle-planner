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
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QStatusBar>
#include <QTabWidget>
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

}  // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    buildUi();

    connect(&m_simulationTimer, &QTimer::timeout, this, &MainWindow::onSimulationTick);
    m_simulationTimer.setInterval(16);

    statusBar()->showMessage(QStringLiteral("系统就绪：请设置任务参数后执行A*规划。"));
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
    const osgEarth::GeoPoint start = makeGeoPoint(m_startLon, m_startLat, m_startAlt);
    const osgEarth::GeoPoint goal = makeGeoPoint(m_goalLon, m_goalLat, m_goalAlt);

    if (!start.isValid() || !goal.isValid()) {
        QMessageBox::warning(this, QStringLiteral("坐标错误"), QStringLiteral("起点或终点坐标无效。"));
        return;
    }

    if (m_earthWidget != nullptr) {
        m_earthWidget->setStartPoint(start);
        m_earthWidget->setTargetPoint(goal);
        m_earthWidget->setThreatZones(m_threatZones);
        m_earthWidget->clearMissile();
    }

    mission::AStarAlgorithm::Options options = m_routePlanner.options();
    options.gridStepDeg = m_gridStepSpin->value();
    options.safetyClearanceMeters = m_clearanceSpin->value();

    const QString profile = m_profileCombo->currentText();
    if (profile.contains(QStringLiteral("低空"))) {
        options.altitudeStepMeters = 160.0;
        options.threatPenaltyScale = 17000.0;
    } else if (profile.contains(QStringLiteral("高空"))) {
        options.altitudeStepMeters = 360.0;
        options.safetyClearanceMeters = std::max(options.safetyClearanceMeters, 520.0);
        options.threatPenaltyScale = 9500.0;
    } else {
        options.altitudeStepMeters = 230.0;
        options.threatPenaltyScale = 13000.0;
    }

    if (!m_threatPenaltyCheck->isChecked()) {
        options.threatPenaltyScale = 0.0;
    }

    m_routePlanner.setOptions(options);
    m_routePlanner.setThreatZones(m_threatZones);
    const mission::RoutePlanResult planResult = m_routePlanner.planRoute(start, goal);

    refreshMetrics(planResult.metrics);

    if (!planResult.metrics.success || planResult.route.size() < 2) {
        m_lastRoute.clear();
        if (m_earthWidget != nullptr) {
            m_earthWidget->setPlannedRoute({});
        }

        QMessageBox::warning(
            this,
            QStringLiteral("规划失败"),
            QString::fromStdString(planResult.metrics.message));
        return;
    }

    m_lastRoute = planResult.route;
    if (m_earthWidget != nullptr) {
        m_earthWidget->setPlannedRoute(m_lastRoute);
        m_earthWidget->focusOnRoute(m_lastRoute);
        m_earthWidget->clearImpactEffect();
    }

    refreshSceneDataSourceLabel();

    statusBar()->showMessage(
        QStringLiteral("A*规划完成：航迹点 %1 个").arg(static_cast<int>(m_lastRoute.size())),
        3000);
}

void MainWindow::onStartSimulation() {
    if (m_lastRoute.size() < 2) {
        // If route is missing, try to generate it so one-click simulation is possible.
        onPlanRoute();
        if (m_lastRoute.size() < 2) {
            return;
        }
    }

    m_missileSim.setRoute(m_lastRoute);
    m_missileSim.start(m_speedSpin->value());

    if (!m_missileSim.isRunning()) {
        QMessageBox::warning(this, QStringLiteral("推演失败"), QStringLiteral("导弹推演初始化失败，请检查航迹。"));
        return;
    }

    if (m_earthWidget != nullptr) {
        m_earthWidget->setFollowMissile(m_followMissileCheck != nullptr && m_followMissileCheck->isChecked());
        m_earthWidget->clearImpactEffect();
        m_earthWidget->setMissilePosition(m_lastRoute.front());
        m_earthWidget->focusOnPoint(m_lastRoute.front(), 140000.0);
    }

    m_tickClock.restart();
    m_lastTickMs = 0;

    m_simProgressValue->setText(QStringLiteral("0.0%"));
    const auto& simState = m_missileSim.state();
    const double simRate = std::max(1.0, m_speedSpin->value() * m_timeScaleSpin->value());
    const double etaSeconds = simState.totalMeters / simRate;
    m_etaValue->setText(QStringLiteral("%1").arg(etaSeconds, 0, 'f', 1));
    m_phaseValue->setText(phaseToText(simState.phase));
    m_currentSpeedValue->setText(QStringLiteral("%1")
                                     .arg(simState.currentSpeedMetersPerSecond, 0, 'f', 1));

    resetTelemetryPanel();
    if (!m_lastRoute.empty()) {
        updateTelemetryPanel(m_lastRoute.front(), simState, 0.016);
    }

    refreshSceneDataSourceLabel();

    m_simulationTimer.start();
    statusBar()->showMessage(QStringLiteral("三维推演进行中..."));
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

    osgEarth::GeoPoint missilePoint;
    const bool ok = m_missileSim.update(scaledDeltaSeconds, missilePoint);
    if (!ok) {
        m_simulationTimer.stop();
        if (!m_lastRoute.empty() && m_earthWidget != nullptr) {
            m_earthWidget->setMissilePosition(m_lastRoute.back());
            m_earthWidget->showImpactEffect(m_lastRoute.back());
            m_earthWidget->focusOnPoint(m_lastRoute.back(), 90000.0);
        }
        m_simProgressValue->setText(QStringLiteral("100.0%"));
        m_etaValue->setText(QStringLiteral("0.0"));
        m_phaseValue->setText(QStringLiteral("命中完成"));
        m_currentSpeedValue->setText(QStringLiteral("0.0"));
        statusBar()->showMessage(QStringLiteral("推演结束。"), 3000);
        return;
    }

    if (m_earthWidget != nullptr) {
        m_earthWidget->setMissilePosition(missilePoint);
    }

    const auto& simState = m_missileSim.state();
    const double progress = simState.totalMeters > 1e-6
                                ? std::clamp(simState.traveledMeters / simState.totalMeters * 100.0, 0.0, 100.0)
                                : 0.0;
    m_simProgressValue->setText(QStringLiteral("%1%")
                                    .arg(progress, 0, 'f', 1));

    const double simRate = std::max(1.0, m_speedSpin->value() * m_timeScaleSpin->value());
    const double remaining = std::max(0.0, simState.totalMeters - simState.traveledMeters);
    m_etaValue->setText(QStringLiteral("%1").arg(remaining / simRate, 0, 'f', 1));
    m_phaseValue->setText(phaseToText(simState.phase));
    m_currentSpeedValue->setText(QStringLiteral("%1")
                                     .arg(simState.currentSpeedMetersPerSecond, 0, 'f', 1));

    updateTelemetryPanel(missilePoint, simState, realDeltaSeconds);

    if (!simState.running) {
        m_simulationTimer.stop();
        if (!m_lastRoute.empty() && m_earthWidget != nullptr) {
            m_earthWidget->showImpactEffect(m_lastRoute.back());
            m_earthWidget->focusOnPoint(m_lastRoute.back(), 90000.0);
        }
        m_simProgressValue->setText(QStringLiteral("100.0%"));
        m_etaValue->setText(QStringLiteral("0.0"));
        m_phaseValue->setText(QStringLiteral("命中完成"));
        m_currentSpeedValue->setText(QStringLiteral("0.0"));
        statusBar()->showMessage(QStringLiteral("推演完成，导弹已到达目标。"), 3000);
    }
}

void MainWindow::buildUi() {
    setWindowTitle(QStringLiteral("空面导弹任务规划与验证可视化系统（作战评估版）"));
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
        "QDoubleSpinBox, QComboBox, QListWidget { background: #0f1b28; border: 1px solid #35506d; border-radius: 3px; color: #e8f3ff; selection-background-color: #2f77aa; }"
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
    panelLayout->setSpacing(8);

    auto* titleLabel = new QLabel(QStringLiteral("态势任务控制台"), panel);
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
                       "data/earth/world.earth\n"
                       "可选: resources/world.earth / default.earth"),
        sceneGroup);
    offlineHint->setWordWrap(true);
    offlineHint->setStyleSheet(QStringLiteral("color: #8fb8dd;"));
    sceneLayout->addRow(QStringLiteral("地球模型"), m_globeModeCombo);
    sceneLayout->addRow(QStringLiteral("数据源"), m_sceneDataSourceValue);
    sceneLayout->addRow(QStringLiteral("离线放置"), offlineHint);
    panelLayout->addWidget(sceneGroup);

    auto* tabs = new QTabWidget(panel);
    tabs->setDocumentMode(true);

    auto* missionTab = new QWidget(tabs);
    auto* missionLayout = new QVBoxLayout(missionTab);
    missionLayout->setContentsMargins(6, 8, 6, 8);
    missionLayout->setSpacing(8);

    auto* startGoalGroup = new QGroupBox(QStringLiteral("任务起点 / 目标点"), missionTab);
    auto* startGoalLayout = new QGridLayout(startGoalGroup);

    m_startLon = createSpinBox(-180.0, 180.0, 112.3500, 6, 0.01, startGoalGroup);
    m_startLat = createSpinBox(-90.0, 90.0, 34.7000, 6, 0.01, startGoalGroup);
    m_startAlt = createSpinBox(0.0, 30000.0, 1200.0, 1, 50.0, startGoalGroup);

    m_goalLon = createSpinBox(-180.0, 180.0, 113.3000, 6, 0.01, startGoalGroup);
    m_goalLat = createSpinBox(-90.0, 90.0, 35.1500, 6, 0.01, startGoalGroup);
    m_goalAlt = createSpinBox(0.0, 30000.0, 1600.0, 1, 50.0, startGoalGroup);

    int row = 0;
    startGoalLayout->addWidget(new QLabel(QStringLiteral("起点经度"), startGoalGroup), row, 0);
    startGoalLayout->addWidget(m_startLon, row, 1);
    ++row;
    startGoalLayout->addWidget(new QLabel(QStringLiteral("起点纬度"), startGoalGroup), row, 0);
    startGoalLayout->addWidget(m_startLat, row, 1);
    ++row;
    startGoalLayout->addWidget(new QLabel(QStringLiteral("起点高度(m)"), startGoalGroup), row, 0);
    startGoalLayout->addWidget(m_startAlt, row, 1);
    ++row;
    startGoalLayout->addWidget(new QLabel(QStringLiteral("目标经度"), startGoalGroup), row, 0);
    startGoalLayout->addWidget(m_goalLon, row, 1);
    ++row;
    startGoalLayout->addWidget(new QLabel(QStringLiteral("目标纬度"), startGoalGroup), row, 0);
    startGoalLayout->addWidget(m_goalLat, row, 1);
    ++row;
    startGoalLayout->addWidget(new QLabel(QStringLiteral("目标高度(m)"), startGoalGroup), row, 0);
    startGoalLayout->addWidget(m_goalAlt, row, 1);

    auto* tacticalGroup = new QGroupBox(QStringLiteral("战术参数"), missionTab);
    auto* tacticalLayout = new QFormLayout(tacticalGroup);
    m_missileTypeCombo = new QComboBox(tacticalGroup);
    m_missileTypeCombo->addItems({
        QStringLiteral("高亚音速巡航导弹"),
        QStringLiteral("超音速突防导弹"),
        QStringLiteral("高超滑翔弹头")});

    m_profileCombo = new QComboBox(tacticalGroup);
    m_profileCombo->addItems({
        QStringLiteral("低空隐蔽突防"),
        QStringLiteral("高空快速突防"),
        QStringLiteral("混合剖面突防")});

    m_clearanceSpin = createSpinBox(80.0, 2000.0, 320.0, 0, 20.0, tacticalGroup);
    m_gridStepSpin = createSpinBox(0.01, 0.2, 0.04, 3, 0.005, tacticalGroup);
    m_threatPenaltyCheck = new QCheckBox(QStringLiteral("启用威胁区软惩罚"), tacticalGroup);
    m_threatPenaltyCheck->setChecked(true);

    tacticalLayout->addRow(QStringLiteral("导弹型号"), m_missileTypeCombo);
    tacticalLayout->addRow(QStringLiteral("突防剖面"), m_profileCombo);
    tacticalLayout->addRow(QStringLiteral("最低离地裕度(m)"), m_clearanceSpin);
    tacticalLayout->addRow(QStringLiteral("网格分辨率(°)"), m_gridStepSpin);
    tacticalLayout->addRow(QStringLiteral(""), m_threatPenaltyCheck);

    auto* threatGroup = new QGroupBox(QStringLiteral("雷达威胁区"), missionTab);
    auto* threatLayout = new QGridLayout(threatGroup);

    m_threatLon = createSpinBox(-180.0, 180.0, 112.8000, 6, 0.01, threatGroup);
    m_threatLat = createSpinBox(-90.0, 90.0, 34.9300, 6, 0.01, threatGroup);
    m_threatRadius = createSpinBox(500.0, 120000.0, 22000.0, 1, 500.0, threatGroup);
    m_threatMaxAlt = createSpinBox(100.0, 20000.0, 3000.0, 1, 100.0, threatGroup);

    row = 0;
    threatLayout->addWidget(new QLabel(QStringLiteral("中心经度"), threatGroup), row, 0);
    threatLayout->addWidget(m_threatLon, row, 1);
    ++row;
    threatLayout->addWidget(new QLabel(QStringLiteral("中心纬度"), threatGroup), row, 0);
    threatLayout->addWidget(m_threatLat, row, 1);
    ++row;
    threatLayout->addWidget(new QLabel(QStringLiteral("半径(m)"), threatGroup), row, 0);
    threatLayout->addWidget(m_threatRadius, row, 1);
    ++row;
    threatLayout->addWidget(new QLabel(QStringLiteral("高度上限(m)"), threatGroup), row, 0);
    threatLayout->addWidget(m_threatMaxAlt, row, 1);

    auto* addThreatButton = new QPushButton(QStringLiteral("添加威胁区"), threatGroup);
    auto* clearThreatButton = new QPushButton(QStringLiteral("清空威胁区"), threatGroup);
    threatLayout->addWidget(addThreatButton, row + 1, 0);
    threatLayout->addWidget(clearThreatButton, row + 1, 1);

    m_threatList = new QListWidget(threatGroup);
    threatLayout->addWidget(m_threatList, row + 2, 0, 1, 2);

    missionLayout->addWidget(startGoalGroup);
    missionLayout->addWidget(tacticalGroup);
    missionLayout->addWidget(threatGroup);
    missionLayout->addStretch(1);

    auto* executeTab = new QWidget(tabs);
    auto* executeLayout = new QVBoxLayout(executeTab);
    executeLayout->setContentsMargins(6, 8, 6, 8);
    executeLayout->setSpacing(8);

    auto* actionGroup = new QGroupBox(QStringLiteral("规划与推演"), executeTab);
    auto* actionLayout = new QVBoxLayout(actionGroup);

    auto* planButton = new QPushButton(QStringLiteral("执行A*路径规划"), actionGroup);
    auto* simButton = new QPushButton(QStringLiteral("开始三维推演"), actionGroup);

    auto* speedLayout = new QFormLayout;
    m_speedSpin = createSpinBox(30.0, 1200.0, 250.0, 1, 10.0, actionGroup);
    m_timeScaleSpin = createSpinBox(1.0, 240.0, 45.0, 1, 1.0, actionGroup);
    m_followMissileCheck = new QCheckBox(QStringLiteral("跟随导弹视角（可随时鼠标接管）"), actionGroup);
    m_followMissileCheck->setChecked(false);

    speedLayout->addRow(QStringLiteral("导弹速度(m/s)"), m_speedSpin);
    speedLayout->addRow(QStringLiteral("推演倍率(x)"), m_timeScaleSpin);

    actionLayout->addWidget(planButton);
    actionLayout->addWidget(simButton);
    actionLayout->addLayout(speedLayout);
    actionLayout->addWidget(m_followMissileCheck);

    auto* metricGroup = new QGroupBox(QStringLiteral("任务指标"), executeTab);
    auto* metricLayout = new QFormLayout(metricGroup);

    m_planTimeValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_pathLengthValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_nodesValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_simProgressValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_etaValue = new QLabel(QStringLiteral("--"), metricGroup);
    m_phaseValue = new QLabel(QStringLiteral("待命"), metricGroup);
    m_currentSpeedValue = new QLabel(QStringLiteral("0.0"), metricGroup);

    metricLayout->addRow(QStringLiteral("规划耗时(ms)"), m_planTimeValue);
    metricLayout->addRow(QStringLiteral("航程长度(m)"), m_pathLengthValue);
    metricLayout->addRow(QStringLiteral("扩展节点数"), m_nodesValue);
    metricLayout->addRow(QStringLiteral("推演进度"), m_simProgressValue);
    metricLayout->addRow(QStringLiteral("预计剩余(s)"), m_etaValue);
    metricLayout->addRow(QStringLiteral("飞行阶段"), m_phaseValue);
    metricLayout->addRow(QStringLiteral("当前速度(m/s)"), m_currentSpeedValue);

    auto* interactionHint = new QGroupBox(QStringLiteral("交互说明"), executeTab);
    auto* hintLayout = new QVBoxLayout(interactionHint);
    auto* hintLabel = new QLabel(
        QStringLiteral("左键拖动：旋转地球\n滚轮：缩放\n右键拖动：平移视角\n按住鼠标操作时将暂时关闭自动跟随"),
        interactionHint);
    hintLabel->setWordWrap(true);
    hintLayout->addWidget(hintLabel);

    auto* zoomButtonsLayout = new QHBoxLayout;
    auto* zoomOutButton = new QPushButton(QStringLiteral("- 缩小"), executeTab);
    auto* zoomInButton = new QPushButton(QStringLiteral("+ 放大"), executeTab);
    zoomButtonsLayout->addWidget(zoomOutButton);
    zoomButtonsLayout->addWidget(zoomInButton);

    executeLayout->addWidget(actionGroup);
    executeLayout->addWidget(metricGroup);
    executeLayout->addWidget(interactionHint);
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
        QStringLiteral("飞行实时可视化（俯仰/速度/高度/剩余航程）"),
        rightPane);
    auto* telemetryLayout = new QVBoxLayout(telemetryGroup);
    telemetryLayout->setContentsMargins(8, 10, 8, 8);
    m_telemetryWidget = new TelemetryPlotWidget(telemetryGroup);
    telemetryLayout->addWidget(m_telemetryWidget);
    telemetryGroup->setMinimumHeight(250);
    telemetryGroup->setMaximumHeight(360);

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
    connect(zoomInButton, &QPushButton::clicked, this, [this]() {
        if (m_earthWidget != nullptr) {
            m_earthWidget->zoomIn();
        }
    });
    connect(zoomOutButton, &QPushButton::clicked, this, [this]() {
        if (m_earthWidget != nullptr) {
            m_earthWidget->zoomOut();
        }
    });

    connect(m_globeModeCombo, &QComboBox::currentIndexChanged, this, [this](int index) {
        if (m_earthWidget == nullptr) {
            return;
        }

        const auto mode = index == 0
                              ? OsgEarthWidget::GlobeMode::Realistic
                              : OsgEarthWidget::GlobeMode::Presentation;
        m_earthWidget->setGlobeMode(mode);
        refreshSceneDataSourceLabel();
    });

    connect(m_missileTypeCombo, &QComboBox::currentTextChanged, this, [this](const QString& text) {
        if (text.contains(QStringLiteral("高亚音速"))) {
            m_speedSpin->setValue(250.0);
            m_timeScaleSpin->setValue(45.0);
        } else if (text.contains(QStringLiteral("超音速"))) {
            m_speedSpin->setValue(520.0);
            m_timeScaleSpin->setValue(38.0);
        } else {
            m_speedSpin->setValue(900.0);
            m_timeScaleSpin->setValue(30.0);
        }
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
    if (m_threatList == nullptr) {
        return;
    }

    m_threatList->clear();
    for (std::size_t i = 0; i < m_threatZones.size(); ++i) {
        const auto& t = m_threatZones[i];
        m_threatList->addItem(QStringLiteral("#%1 经度:%2 纬度:%3 半径:%4m 高度:%5m")
                                  .arg(static_cast<int>(i + 1))
                                  .arg(t.longitudeDeg, 0, 'f', 3)
                                  .arg(t.latitudeDeg, 0, 'f', 3)
                                  .arg(t.radiusMeters, 0, 'f', 0)
                                  .arg(t.maxAltitudeMeters, 0, 'f', 0));
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
    if (m_sceneDataSourceValue == nullptr) {
        return;
    }

    if (m_earthWidget == nullptr) {
        m_sceneDataSourceValue->setText(QStringLiteral("地图控件未就绪"));
        return;
    }

    m_sceneDataSourceValue->setText(m_earthWidget->realEarthStatusText());
}

void MainWindow::resetTelemetryPanel() {
    m_hasTelemetryPrevPoint = false;
    m_prevTelemetrySpeed = 0.0;
    m_prevTelemetryTime = 0.0;

    if (m_telemetryWidget != nullptr) {
        m_telemetryWidget->clearHistory();
    }
}

void MainWindow::updateTelemetryPanel(
    const osgEarth::GeoPoint& missilePoint,
    const mission::MissileSim::State& simState,
    double realDeltaSeconds) {
    if (m_telemetryWidget == nullptr || !missilePoint.isValid()) {
        return;
    }

    double heading = 0.0;
    double pitch = 0.0;
    if (m_hasTelemetryPrevPoint && m_prevTelemetryPoint.isValid()) {
        heading = headingDegrees(m_prevTelemetryPoint, missilePoint);
        pitch = pitchDegrees(m_prevTelemetryPoint, missilePoint);
    }

    const double safeDelta = std::max(0.001, realDeltaSeconds);
    const double accel = (simState.currentSpeedMetersPerSecond - m_prevTelemetrySpeed) / safeDelta;
    const double remaining = std::max(0.0, simState.totalMeters - simState.traveledMeters);

    TelemetryPlotWidget::Sample sample;
    sample.timeSeconds = simState.elapsedSeconds;
    sample.speedMetersPerSecond = simState.currentSpeedMetersPerSecond;
    sample.altitudeMeters = missilePoint.z();
    sample.pitchDegrees = pitch;
    sample.headingDegrees = heading;
    sample.remainingMeters = remaining;
    sample.accelerationMetersPerSecond2 = m_hasTelemetryPrevPoint ? accel : 0.0;
    sample.phaseText = phaseToText(simState.phase);
    m_telemetryWidget->pushSample(sample);

    m_prevTelemetryPoint = missilePoint;
    m_hasTelemetryPrevPoint = true;
    m_prevTelemetrySpeed = simState.currentSpeedMetersPerSecond;
    m_prevTelemetryTime = simState.elapsedSeconds;
}
