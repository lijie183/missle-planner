#include "ui/FlightReportExporter.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace {

QString missileTypeName(int type) {
    switch (type) {
        case 0: return QStringLiteral("高亚音速");
        case 1: return QStringLiteral("超音速");
        case 2: return QStringLiteral("高超滑翔");
        default: return QStringLiteral("未知");
    }
}

QJsonObject missileToJson(const mission::MissileConfig& missile) {
    QJsonObject obj;
    obj[QStringLiteral("id")] = QString::fromStdString(missile.id);
    obj[QStringLiteral("name")] = QString::fromStdString(missile.name);
    obj[QStringLiteral("startLonDeg")] = missile.startLonDeg;
    obj[QStringLiteral("startLatDeg")] = missile.startLatDeg;
    obj[QStringLiteral("startAltMeters")] = missile.startAltMeters;
    obj[QStringLiteral("missileType")] = missile.missileType;
    obj[QStringLiteral("missileTypeName")] = missileTypeName(missile.missileType);
    obj[QStringLiteral("speedMps")] = missile.speedMps;
    return obj;
}

QJsonObject targetToJson(const mission::TargetConfig& target) {
    QJsonObject obj;
    obj[QStringLiteral("id")] = QString::fromStdString(target.id);
    obj[QStringLiteral("name")] = QString::fromStdString(target.name);
    obj[QStringLiteral("lonDeg")] = target.lonDeg;
    obj[QStringLiteral("latDeg")] = target.latDeg;
    obj[QStringLiteral("altMeters")] = target.altMeters;
    obj[QStringLiteral("priority")] = target.priority;
    return obj;
}

QJsonObject threatToJson(const mission::ThreatZone& threat) {
    QJsonObject obj;
    obj[QStringLiteral("longitudeDeg")] = threat.longitudeDeg;
    obj[QStringLiteral("latitudeDeg")] = threat.latitudeDeg;
    obj[QStringLiteral("radiusMeters")] = threat.radiusMeters;
    obj[QStringLiteral("minAltitudeMeters")] = threat.minAltitudeMeters;
    obj[QStringLiteral("maxAltitudeMeters")] = threat.maxAltitudeMeters;
    return obj;
}

QJsonObject metricsToJson(const mission::PlanMetrics& metrics) {
    QJsonObject obj;
    obj[QStringLiteral("success")] = metrics.success;
    obj[QStringLiteral("message")] = QString::fromStdString(metrics.message);
    obj[QStringLiteral("planningTimeMs")] = metrics.planningTimeMs;
    obj[QStringLiteral("pathLengthMeters")] = metrics.pathLengthMeters;
    obj[QStringLiteral("expandedNodes")] = metrics.expandedNodes;
    obj[QStringLiteral("visitedNodes")] = metrics.visitedNodes;
    return obj;
}

QJsonObject assignmentToJson(const mission::Assignment& assignment) {
    QJsonObject obj;
    obj[QStringLiteral("missileIndex")] = assignment.missileIndex;
    obj[QStringLiteral("targetIndex")] = assignment.targetIndex;
    obj[QStringLiteral("planned")] = assignment.planned;
    obj[QStringLiteral("metrics")] = metricsToJson(assignment.planResult.metrics);
    QJsonArray routeWaypoints;
    for (const auto& point : assignment.planResult.route) {
        QJsonObject geo;
        geo[QStringLiteral("longitudeDeg")] = point.x();
        geo[QStringLiteral("latitudeDeg")] = point.y();
        geo[QStringLiteral("altitudeMeters")] = point.z();
        routeWaypoints.append(geo);
    }
    obj[QStringLiteral("routeWaypoints")] = routeWaypoints;
    return obj;
}

QJsonObject telemetryToJson(const mission::TelemetrySample& sample) {
    QJsonObject obj;
    obj[QStringLiteral("timeSeconds")] = sample.timeSeconds;
    obj[QStringLiteral("speedMetersPerSecond")] = sample.speedMetersPerSecond;
    obj[QStringLiteral("altitudeMeters")] = sample.altitudeMeters;
    obj[QStringLiteral("pitchDegrees")] = sample.pitchDegrees;
    obj[QStringLiteral("headingDegrees")] = sample.headingDegrees;
    obj[QStringLiteral("remainingMeters")] = sample.remainingMeters;
    obj[QStringLiteral("accelerationMetersPerSecond2")] = sample.accelerationMetersPerSecond2;
    return obj;
}

QString escapeHtml(const QString& text) {
    QString result = text;
    result.replace('&', QStringLiteral("&amp;"));
    result.replace('<', QStringLiteral("&lt;"));
    result.replace('>', QStringLiteral("&gt;"));
    result.replace('"', QStringLiteral("&quot;"));
    return result;
}

}  // namespace

namespace mission {

QString buildFlightReportHtml(const FlightReportData& report) {
    QJsonObject root;
    root[QStringLiteral("title")] = report.title;
    root[QStringLiteral("generatedAt")] = report.generatedAt.toString(Qt::ISODate);
    root[QStringLiteral("missiles")] = QJsonArray();
    root[QStringLiteral("targets")] = QJsonArray();
    root[QStringLiteral("threats")] = QJsonArray();
    root[QStringLiteral("routes")] = QJsonArray();
    root[QStringLiteral("planningResult")] = QJsonObject();
    root[QStringLiteral("successCount")] = report.planningResult.successCount;
    root[QStringLiteral("failureCount")] = report.planningResult.failureCount;
    root[QStringLiteral("successRate")] = report.planningResult.successRate;

    QJsonArray missiles;
    for (const auto& missile : report.missiles) {
        missiles.append(missileToJson(missile));
    }
    root[QStringLiteral("missiles")] = missiles;

    QJsonArray targets;
    for (const auto& target : report.targets) {
        targets.append(targetToJson(target));
    }
    root[QStringLiteral("targets")] = targets;

    QJsonArray threats;
    for (const auto& threat : report.threats) {
        threats.append(threatToJson(threat));
    }
    root[QStringLiteral("threats")] = threats;

    QJsonObject planningResult;
    planningResult[QStringLiteral("message")] = QString::fromStdString(report.planningResult.message);
    planningResult[QStringLiteral("totalPlanningTimeMs")] = report.planningResult.totalPlanningTimeMs;
    planningResult[QStringLiteral("successCount")] = report.planningResult.successCount;
    planningResult[QStringLiteral("failureCount")] = report.planningResult.failureCount;
    planningResult[QStringLiteral("successRate")] = report.planningResult.successRate;
    QJsonArray assignments;
    for (const auto& assignment : report.planningResult.assignments) {
        QJsonObject item = assignmentToJson(assignment);
        item[QStringLiteral("statusText")] = assignment.planned ? QStringLiteral("规划成功")
                                                                 : (assignment.targetIndex < 0 ? QStringLiteral("未分配") : QStringLiteral("规划失败"));
        assignments.append(item);
    }
    planningResult[QStringLiteral("assignments")] = assignments;
    root[QStringLiteral("planningResult")] = planningResult;

    QJsonArray routes;
    for (const auto& route : report.routes) {
        QJsonObject routeObj;
        routeObj[QStringLiteral("missile")] = missileToJson(route.missile);
        routeObj[QStringLiteral("target")] = targetToJson(route.target);
        routeObj[QStringLiteral("assignment")] = assignmentToJson(route.assignment);
        QJsonArray telemetry;
        for (const auto& sample : route.telemetry) {
            telemetry.append(telemetryToJson(sample));
        }
        routeObj[QStringLiteral("telemetry")] = telemetry;
        QJsonArray waypoints;
        for (const auto& point : route.routeWaypoints) {
            QJsonObject geo;
            geo[QStringLiteral("longitudeDeg")] = point.x();
            geo[QStringLiteral("latitudeDeg")] = point.y();
            geo[QStringLiteral("altitudeMeters")] = point.z();
            waypoints.append(geo);
        }
        routeObj[QStringLiteral("routeWaypoints")] = waypoints;
        routes.append(routeObj);
    }
    root[QStringLiteral("routes")] = routes;

    QString json = QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Compact));
    json.replace(QStringLiteral("</"), QStringLiteral("<\\/"));

    QString html;
    html.reserve(180000);
    html += QStringLiteral(R"COPILOTHTML(<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>)COPILOTHTML");
    html += escapeHtml(report.title);
    html += QStringLiteral(R"COPILOTHTML(</title>
<style>
:root {
  --bg0: #08111c;
  --bg1: #0f1b2a;
  --card: rgba(16, 28, 42, 0.94);
  --card2: rgba(21, 35, 52, 0.95);
  --line: rgba(119, 175, 227, 0.16);
  --text: #ebf4ff;
  --muted: #9eb4c8;
  --accent: #59d0ff;
  --accent2: #7ef0aa;
  --shadow: 0 24px 80px rgba(0, 0, 0, 0.38);
}
* { box-sizing: border-box; }
body {
  margin: 0;
  color: var(--text);
  font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
  background:
    radial-gradient(circle at top left, rgba(39, 122, 183, 0.22), transparent 32%),
    radial-gradient(circle at top right, rgba(42, 189, 142, 0.16), transparent 30%),
    linear-gradient(180deg, var(--bg0), var(--bg1) 48%, #07101a 100%);
}
.shell {
  max-width: 1600px;
  margin: 0 auto;
  padding: 28px 22px 42px;
}
.hero {
  position: sticky;
  top: 0;
  z-index: 20;
  backdrop-filter: blur(10px);
  background: linear-gradient(180deg, rgba(7, 16, 26, 0.94), rgba(7, 16, 26, 0.72));
  border: 1px solid var(--line);
  border-radius: 22px;
  box-shadow: var(--shadow);
  padding: 22px 24px;
  margin-bottom: 18px;
}
.hero h1 {
  margin: 0 0 10px;
  font-size: 28px;
}
.meta {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  color: var(--muted);
  font-size: 13px;
}
.chip {
  padding: 7px 12px;
  background: rgba(92, 145, 192, 0.12);
  border: 1px solid rgba(122, 170, 215, 0.18);
  border-radius: 999px;
}
.card {
  background: var(--card);
  border: 1px solid var(--line);
  border-radius: 20px;
  box-shadow: var(--shadow);
  overflow: hidden;
}
.card-header {
  padding: 16px 18px 12px;
  display: flex;
  justify-content: space-between;
  gap: 12px;
  align-items: center;
  border-bottom: 1px solid rgba(125, 176, 224, 0.14);
  background: linear-gradient(180deg, rgba(22, 38, 58, 0.9), rgba(15, 26, 39, 0.76));
}
.card-body { padding: 16px 18px 18px; }
.section { margin-top: 18px; }
.section-title { display: flex; justify-content: space-between; align-items: center; gap: 12px; margin-bottom: 10px; }
.section-title h2 { margin: 0; font-size: 20px; }
.section-title .subtle { color: var(--muted); font-size: 13px; }
.grid-3, .cards-grid, .chart-grid { display: grid; gap: 14px; }
.grid-3, .cards-grid { grid-template-columns: repeat(3, minmax(0, 1fr)); }
.chart-grid { grid-template-columns: repeat(3, minmax(0, 1fr)); }
.kpi {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 12px;
}
.kpi-item, .entity-card {
  padding: 14px 16px;
  border-radius: 16px;
  background: rgba(255, 255, 255, 0.03);
  border: 1px solid rgba(124, 173, 217, 0.12);
}
.kpi-label { color: var(--muted); font-size: 12px; margin-bottom: 8px; }
.kpi-value { font-size: 22px; font-weight: 700; }
.entity-title { font-size: 15px; font-weight: 700; margin-bottom: 10px; }
.kv { display: grid; grid-template-columns: 1fr 1fr; gap: 8px 14px; font-size: 13px; }
.kv div { color: var(--muted); }
.kv span { color: var(--text); }
.chart { min-height: 280px; }
.chart-body { position: relative; }
.chart-canvas { width: 100%; height: 230px; display: block; }
.tooltip {
  position: absolute;
  pointer-events: none;
  transform: translate(10px, 10px);
  min-width: 190px;
  max-width: 280px;
  padding: 10px 12px;
  border-radius: 12px;
  background: rgba(8, 17, 28, 0.95);
  border: 1px solid rgba(108, 164, 214, 0.26);
  box-shadow: 0 16px 40px rgba(0, 0, 0, 0.3);
  opacity: 0;
  transition: opacity .12s ease;
  font-size: 12px;
  line-height: 1.45;
}
.tooltip.visible { opacity: 1; }
.tooltip .name { font-weight: 700; margin-bottom: 6px; }
.route-tabs { display: flex; flex-wrap: wrap; gap: 10px; margin-bottom: 12px; }
.route-tab {
  appearance: none;
  border: 1px solid rgba(122, 176, 223, 0.18);
  background: rgba(19, 32, 47, 0.84);
  color: var(--text);
  border-radius: 999px;
  padding: 10px 15px;
  font-size: 13px;
  cursor: pointer;
}
.route-tab.active { background: linear-gradient(135deg, #1e88c8, #3dbd92); border-color: transparent; }
.route-panel { display: none; }
.route-panel.active { display: block; }
.waypoints { width: 100%; border-collapse: collapse; margin-top: 12px; font-size: 12px; }
.waypoints th, .waypoints td { border-bottom: 1px solid rgba(124, 173, 217, 0.14); padding: 8px 6px; text-align: left; }
.waypoints th { color: #b1cde4; font-weight: 700; }
.empty {
  padding: 16px;
  border-radius: 16px;
  background: rgba(255, 255, 255, 0.03);
  border: 1px dashed rgba(124, 173, 217, 0.16);
  color: var(--muted);
}
@media (max-width: 1200px) {
  .grid-3, .cards-grid, .chart-grid, .kpi { grid-template-columns: repeat(2, minmax(0, 1fr)); }
}
@media (max-width: 760px) {
  .grid-3, .cards-grid, .chart-grid, .kpi { grid-template-columns: 1fr; }
  .hero { border-radius: 18px; }
  .shell { padding: 18px 12px 30px; }
}
</style>
</head>
<body>
<div class="shell">
  <div class="hero">
    <h1>)COPILOTHTML");
    html += escapeHtml(report.title);
    html += QStringLiteral(R"COPILOTHTML(</h1>
    <div class="meta">
      <span class="chip">生成时间：)COPILOTHTML");
    html += escapeHtml(report.generatedAt.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss")));
    html += QStringLiteral(R"COPILOTHTML(</span>
      <span class="chip">导弹：)COPILOTHTML");
    html += QString::number(report.missiles.size());
    html += QStringLiteral(R"COPILOTHTML(</span>
      <span class="chip">目标：)COPILOTHTML");
    html += QString::number(report.targets.size());
    html += QStringLiteral(R"COPILOTHTML(</span>
      <span class="chip">威胁区：)COPILOTHTML");
    html += QString::number(report.threats.size());
    html += QStringLiteral(R"COPILOTHTML(</span>
      <span class="chip">成功率：)COPILOTHTML");
    html += QString::number(report.planningResult.successRate * 100.0, 'f', 1);
    html += QStringLiteral(R"COPILOTHTML(%</span>
    </div>
  </div>
  <div id="app"></div>
</div>
<script id="flight-report-data" type="application/json">)COPILOTHTML");
    html += json;
    html += QStringLiteral(R"COPILOTHTML(</script>
<script>
(() => {
  const report = JSON.parse(document.getElementById('flight-report-data').textContent);
  const palette = ['#59d0ff', '#7ef0aa', '#ffb86a', '#ff6d7a', '#ad8cff', '#f7d154', '#4ed0c8', '#ff8fb1'];
  const metrics = [
    { key: 'altitudeMeters', title: '高度', unit: 'm', formatter: v => v.toFixed(1) },
    { key: 'remainingMeters', title: '弹目距离', unit: 'm', formatter: v => v.toFixed(1) },
    { key: 'speedMetersPerSecond', title: '速度', unit: 'm/s', formatter: v => v.toFixed(1) },
    { key: 'pitchDegrees', title: '俯仰角', unit: '°', formatter: v => v.toFixed(2) },
    { key: 'headingDegrees', title: '航向角', unit: '°', formatter: v => v.toFixed(2) },
    { key: 'accelerationMetersPerSecond2', title: '加速度', unit: 'm/s²', formatter: v => v.toFixed(2) },
  ];

  const app = document.getElementById('app');
  const fmt = (value, digits = 1) => Number.isFinite(value) ? Number(value).toFixed(digits) : '--';
  const esc = (text) => String(text)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');

  function sectionTitle(title, subtitle) {
    return `<div class="section-title"><h2>${esc(title)}</h2><div class="subtle">${esc(subtitle || '')}</div></div>`;
  }

  function computeRange(seriesList) {
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const series of seriesList) {
      for (const point of series.samples || []) {
        if (!Number.isFinite(point.t) || !Number.isFinite(point.v)) continue;
        minX = Math.min(minX, point.t);
        maxX = Math.max(maxX, point.t);
        minY = Math.min(minY, point.v);
        maxY = Math.max(maxY, point.v);
      }
    }
    if (!Number.isFinite(minX) || !Number.isFinite(maxX)) return null;
    if (minX === maxX) maxX = minX + 1;
    if (minY === maxY) {
      const pad = Math.max(1, Math.abs(minY) * 0.1);
      minY -= pad;
      maxY += pad;
    } else {
      const pad = (maxY - minY) * 0.12;
      minY -= pad;
      maxY += pad;
    }
    return { minX, maxX, minY, maxY };
  }

  function chartCard(metric, seriesList, emptyText) {
    const card = document.createElement('div');
    card.className = 'card chart';
    card.innerHTML = `
      <div class="card-header">
        <h3>${esc(metric.title)}</h3>
        <span class="subtle">单位：${esc(metric.unit)}</span>
      </div>
      <div class="card-body chart-body">
        <canvas class="chart-canvas"></canvas>
        <div class="tooltip"></div>
      </div>`;
    renderChart(card.querySelector('canvas'), card.querySelector('.tooltip'), metric, seriesList, emptyText);
    return card;
  }

  function renderChart(canvas, tooltip, metric, seriesList, emptyText) {
    const ctx = canvas.getContext('2d');
    const state = { reveal: 0, hoverX: null };

    function resize() {
      const dpr = window.devicePixelRatio || 1;
      const cssWidth = canvas.clientWidth || 400;
      const cssHeight = canvas.clientHeight || 230;
      canvas.width = Math.max(1, Math.floor(cssWidth * dpr));
      canvas.height = Math.max(1, Math.floor(cssHeight * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      draw();
    }

    function draw() {
      const width = canvas.clientWidth || 400;
      const height = canvas.clientHeight || 230;
      ctx.clearRect(0, 0, width, height);
      const range = computeRange(seriesList);
      if (!range) {
        ctx.fillStyle = '#9eb4c8';
        ctx.font = '13px Microsoft YaHei, sans-serif';
        ctx.fillText(emptyText || '暂无数据', 18, 32);
        return;
      }
      const pad = { left: 54, right: 16, top: 16, bottom: 36 };
      const plot = {
        left: pad.left,
        top: pad.top,
        width: Math.max(1, width - pad.left - pad.right),
        height: Math.max(1, height - pad.top - pad.bottom),
      };
      plot.right = plot.left + plot.width;
      plot.bottom = plot.top + plot.height;

      ctx.fillStyle = 'rgba(255,255,255,0.02)';
      ctx.fillRect(plot.left, plot.top, plot.width, plot.height);
      ctx.strokeStyle = 'rgba(120, 182, 255, 0.12)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let i = 0; i <= 4; ++i) {
        const y = plot.top + plot.height * i / 4;
        ctx.moveTo(plot.left, y);
        ctx.lineTo(plot.right, y);
      }
      for (let i = 0; i <= 4; ++i) {
        const x = plot.left + plot.width * i / 4;
        ctx.moveTo(x, plot.top);
        ctx.lineTo(x, plot.bottom);
      }
      ctx.stroke();

      ctx.fillStyle = '#9eb4c8';
      ctx.font = '11px Microsoft YaHei, sans-serif';
      for (let i = 0; i <= 4; ++i) {
        const y = plot.bottom - plot.height * i / 4;
        const value = range.minY + (range.maxY - range.minY) * i / 4;
        ctx.fillText(metric.formatter(value), 8, y + 4);
      }
      for (let i = 0; i <= 4; ++i) {
        const x = plot.left + plot.width * i / 4;
        const value = range.minX + (range.maxX - range.minX) * i / 4;
        ctx.fillText(fmt(value, value < 10 ? 1 : 0), x - 10, plot.bottom + 18);
      }

      for (const series of seriesList) {
        const points = (series.samples || []).map(sample => {
          const nx = (sample.t - range.minX) / Math.max(1e-9, range.maxX - range.minX);
          const ny = (sample.v - range.minY) / Math.max(1e-9, range.maxY - range.minY);
          return { x: plot.left + nx * plot.width, y: plot.bottom - ny * plot.height, t: sample.t, v: sample.v };
        });
        if (points.length < 1) continue;

        ctx.save();
        ctx.beginPath();
        ctx.strokeStyle = series.color;
        ctx.lineWidth = 2.2;
        ctx.shadowColor = series.color;
        ctx.shadowBlur = 8;
        points.forEach((point, index) => {
          const visible = index / Math.max(1, points.length - 1);
          if (visible > state.reveal) return;
          if (index === 0) ctx.moveTo(point.x, point.y);
          else ctx.lineTo(point.x, point.y);
        });
        ctx.stroke();
        ctx.shadowBlur = 0;
        for (const point of points) {
          const visible = (point.x - plot.left) / Math.max(1, plot.width);
          if (visible > state.reveal) continue;
          ctx.beginPath();
          ctx.fillStyle = series.color;
          ctx.arc(point.x, point.y, 2.7, 0, Math.PI * 2);
          ctx.fill();
        }
        ctx.restore();
      }

      if (state.hoverX != null) {
        ctx.strokeStyle = 'rgba(255,255,255,0.24)';
        ctx.beginPath();
        ctx.moveTo(state.hoverX, plot.top);
        ctx.lineTo(state.hoverX, plot.bottom);
        ctx.stroke();
      }
    }

    function onMove(event) {
      const rect = canvas.getBoundingClientRect();
      const x = event.clientX - rect.left;
      const y = event.clientY - rect.top;
      const range = computeRange(seriesList);
      if (!range) {
        tooltip.classList.remove('visible');
        return;
      }
      const plot = { left: 54, top: 16, right: rect.width - 16, bottom: rect.height - 36 };
      plot.width = plot.right - plot.left;
      plot.height = plot.bottom - plot.top;
      if (x < plot.left || x > plot.right || y < plot.top || y > plot.bottom) {
        tooltip.classList.remove('visible');
        state.hoverX = null;
        draw();
        return;
      }
      const hoverTime = range.minX + ((x - plot.left) / Math.max(1, plot.width)) * (range.maxX - range.minX);
      let labelTime = hoverTime;
      const lines = [];
      let closestDistance = Infinity;
      for (const series of seriesList) {
        if (!series.samples || series.samples.length < 1) continue;
        let best = series.samples[0];
        let bestDistance = Math.abs(best.t - hoverTime);
        for (const sample of series.samples) {
          const distance = Math.abs(sample.t - hoverTime);
          if (distance < bestDistance) {
            best = sample;
            bestDistance = distance;
          }
        }
        if (bestDistance < closestDistance) {
          closestDistance = bestDistance;
          labelTime = best.t;
          state.hoverX = plot.left + ((best.t - range.minX) / Math.max(1e-9, range.maxX - range.minX)) * plot.width;
        }
        lines.push(`<div><span style="color:${series.color}">●</span> ${esc(series.name)}：${metric.formatter(best.v)} ${esc(metric.unit)}</div>`);
      }
      tooltip.innerHTML = `<div class="name">${esc(metric.title)} / ${fmt(labelTime, labelTime < 10 ? 2 : 1)} s</div>${lines.join('')}`;
      tooltip.style.left = `${Math.min(rect.width - 260, Math.max(0, x))}px`;
      tooltip.style.top = `${Math.max(0, y)}px`;
      tooltip.classList.add('visible');
      draw();
    }

    canvas.addEventListener('mousemove', onMove);
    canvas.addEventListener('mouseleave', () => { tooltip.classList.remove('visible'); state.hoverX = null; draw(); });
    window.addEventListener('resize', resize, { passive: true });

    const start = performance.now();
    function animate(now) {
      state.reveal = Math.min(1, (now - start) / 900);
      draw();
      if (state.reveal < 1) requestAnimationFrame(animate);
    }
    resize();
    requestAnimationFrame(animate);
  }

)COPILOTHTML");
    html += QStringLiteral(R"COPILOTHTML(

  function entityCard(title, kv) {
    return `<div class="entity-card"><div class="entity-title">${esc(title)}</div><div class="kv">${kv}</div></div>`;
  }

  function wpTable(waypoints) {
    if (!waypoints || waypoints.length === 0) {
      return '<div class="empty">暂无航路点数据</div>';
    }
    const rows = waypoints.map((point, index) => `<tr><td>${index + 1}</td><td>${fmt(point.longitudeDeg, 4)}</td><td>${fmt(point.latitudeDeg, 4)}</td><td>${fmt(point.altitudeMeters, 0)}</td></tr>`).join('');
    return `<table class="waypoints"><thead><tr><th>#</th><th>经度</th><th>纬度</th><th>高度(m)</th></tr></thead><tbody>${rows}</tbody></table>`;
  }

  const missiles = report.missiles || [];
  const targets = report.targets || [];
  const threats = report.threats || [];
  const routes = report.routes || [];

  const missionCards = document.createElement('section');
  missionCards.className = 'section';
  missionCards.innerHTML = sectionTitle('任务总览', '导弹、目标和威胁区的详细摘要');
  missionCards.insertAdjacentHTML('beforeend', `
    <div class="grid-3">
      <div class="card"><div class="card-header"><h3>任务概览</h3><span class="subtle">全局统计</span></div><div class="card-body">
        <div class="kpi">
          <div class="kpi-item"><div class="kpi-label">导弹数</div><div class="kpi-value">${missiles.length}</div></div>
          <div class="kpi-item"><div class="kpi-label">目标数</div><div class="kpi-value">${targets.length}</div></div>
          <div class="kpi-item"><div class="kpi-label">威胁区</div><div class="kpi-value">${threats.length}</div></div>
          <div class="kpi-item"><div class="kpi-label">航线数</div><div class="kpi-value">${routes.length}</div></div>
        </div>
      </div></div>
      <div class="card"><div class="card-header"><h3>规划结果</h3><span class="subtle">${esc(report.planningResult?.message || '')}</span></div><div class="card-body">
        <div class="kpi">
          <div class="kpi-item"><div class="kpi-label">成功数</div><div class="kpi-value">${report.successCount || 0}</div></div>
          <div class="kpi-item"><div class="kpi-label">失败数</div><div class="kpi-value">${report.failureCount || 0}</div></div>
          <div class="kpi-item"><div class="kpi-label">成功率</div><div class="kpi-value">${fmt((report.successRate || 0) * 100, 1)}%</div></div>
          <div class="kpi-item"><div class="kpi-label">耗时(ms)</div><div class="kpi-value">${fmt(report.planningResult?.totalPlanningTimeMs || 0, 2)}</div></div>
        </div>
      </div></div>
      <div class="card"><div class="card-header"><h3>遥测样本</h3><span class="subtle">实时可视化数据</span></div><div class="card-body">
        <div class="kpi">
          <div class="kpi-item"><div class="kpi-label">总样本</div><div class="kpi-value">${routes.reduce((sum, route) => sum + (route.telemetry?.length || 0), 0)}</div></div>
          <div class="kpi-item"><div class="kpi-label">平均航线样本</div><div class="kpi-value">${routes.length ? fmt(routes.reduce((sum, route) => sum + (route.telemetry?.length || 0), 0) / routes.length, 1) : '0.0'}</div></div>
          <div class="kpi-item"><div class="kpi-label">最大航路点</div><div class="kpi-value">${routes.reduce((max, route) => Math.max(max, (route.routeWaypoints || []).length), 0)}</div></div>
          <div class="kpi-item"><div class="kpi-label">生成状态</div><div class="kpi-value">OK</div></div>
        </div>
      </div></div>
    </div>`);
  app.appendChild(missionCards);

  const detailSection = document.createElement('section');
  detailSection.className = 'section';
  detailSection.innerHTML = sectionTitle('任务明细', '导弹、目标与威胁区的结构化数据');
  const detailGrid = document.createElement('div');
  detailGrid.className = 'cards-grid';

  missiles.forEach((missile, index) => {
    detailGrid.insertAdjacentHTML('beforeend', entityCard(`导弹 ${index + 1}`, `
      <div>编号</div><span>${esc(missile.id || '--')}</span>
      <div>名称</div><span>${esc(missile.name || '--')}</span>
      <div>型号</div><span>${esc(missile.missileTypeName || '--')}</span>
      <div>速度</div><span>${fmt(missile.speedMps || 0, 1)} m/s</span>
      <div>起点</div><span>${fmt(missile.startLonDeg || 0, 3)}°, ${fmt(missile.startLatDeg || 0, 3)}°</span>
      <div>高度</div><span>${fmt(missile.startAltMeters || 0, 0)} m</span>`));
  });
  targets.forEach((target, index) => {
    detailGrid.insertAdjacentHTML('beforeend', entityCard(`目标 ${index + 1}`, `
      <div>编号</div><span>${esc(target.id || '--')}</span>
      <div>名称</div><span>${esc(target.name || '--')}</span>
      <div>优先级</div><span>${target.priority ?? '--'}</span>
      <div>位置</div><span>${fmt(target.lonDeg || 0, 3)}°, ${fmt(target.latDeg || 0, 3)}°</span>
      <div>高度</div><span>${fmt(target.altMeters || 0, 0)} m</span>
      <div>航线数</div><span>${routes.filter(route => route.target?.id === target.id).length}</span>`));
  });
  threats.forEach((threat, index) => {
    detailGrid.insertAdjacentHTML('beforeend', entityCard(`威胁区 ${index + 1}`, `
      <div>中心</div><span>${fmt(threat.longitudeDeg || 0, 3)}°, ${fmt(threat.latitudeDeg || 0, 3)}°</span>
      <div>半径</div><span>${fmt(threat.radiusMeters || 0, 0)} m</span>
      <div>高度下限</div><span>${fmt(threat.minAltitudeMeters || 0, 0)} m</span>
      <div>高度上限</div><span>${fmt(threat.maxAltitudeMeters || 0, 0)} m</span>`));
  });
  detailSection.appendChild(detailGrid);
  app.appendChild(detailSection);

  const overallSection = document.createElement('section');
  overallSection.className = 'section';
  overallSection.innerHTML = sectionTitle('总体六图', '所有航线叠加后的六个图表');
  const overallGrid = document.createElement('div');
  overallGrid.className = 'chart-grid';
  metrics.forEach((metric) => {
    const seriesList = routes.map((route, index) => ({
      name: route.missile?.name || `导弹-${index + 1}`,
      color: palette[index % palette.length],
      samples: (route.telemetry || []).map(sample => ({ t: sample.timeSeconds, v: sample[metric.key] }))
    })).filter(series => series.samples.length > 0);
    overallGrid.appendChild(chartCard(metric, seriesList, '暂无总体数据'));
  });
  overallSection.appendChild(overallGrid);
  app.appendChild(overallSection);

  const routeWrapper = document.createElement('section');
  routeWrapper.className = 'section';
  routeWrapper.innerHTML = sectionTitle('单航线详情', '每条航线各自拥有六个图表与独立参数');
  if (routes.length > 0) {
    const tabs = document.createElement('div');
    tabs.className = 'route-tabs';
    routes.forEach((route, index) => {
      const btn = document.createElement('button');
      btn.className = 'route-tab' + (index === 0 ? ' active' : '');
      btn.textContent = route.missile?.name || `航线 ${index + 1}`;
      btn.addEventListener('click', () => activateRoute(index));
      tabs.appendChild(btn);
    });
    routeWrapper.appendChild(tabs);

    routes.forEach((route, index) => {
      const panel = document.createElement('div');
      panel.className = 'route-panel' + (index === 0 ? ' active' : '');
      panel.dataset.index = index;
      panel.innerHTML = `
        <div class="cards-grid">
          <div class="entity-card"><div class="entity-title">导弹</div><div class="kv">
            <div>编号</div><span>${esc(route.missile?.id || '--')}</span>
            <div>名称</div><span>${esc(route.missile?.name || '--')}</span>
            <div>型号</div><span>${esc(route.missile?.missileTypeName || '--')}</span>
            <div>速度</div><span>${fmt(route.missile?.speedMps || 0, 1)} m/s</span>
            <div>起点</div><span>${fmt(route.missile?.startLonDeg || 0, 3)}°, ${fmt(route.missile?.startLatDeg || 0, 3)}°</span>
            <div>高度</div><span>${fmt(route.missile?.startAltMeters || 0, 0)} m</span>
          </div></div>
          <div class="entity-card"><div class="entity-title">目标</div><div class="kv">
            <div>编号</div><span>${esc(route.target?.id || '--')}</span>
            <div>名称</div><span>${esc(route.target?.name || '--')}</span>
            <div>优先级</div><span>${route.target?.priority ?? '--'}</span>
            <div>位置</div><span>${route.target?.id ? `${fmt(route.target.lonDeg || 0, 3)}°, ${fmt(route.target.latDeg || 0, 3)}°` : '--'}</span>
            <div>高度</div><span>${route.target?.id ? `${fmt(route.target.altMeters || 0, 0)} m` : '--'}</span>
            <div>航路点</div><span>${(route.routeWaypoints || []).length}</span>
          </div></div>
          <div class="entity-card"><div class="entity-title">状态</div><div class="kv">
            <div>状态</div><span>${route.assignment?.planned ? (route.failed ? '失效' : (route.completed ? '完成' : '进行中')) : '未规划'}</span>
            <div>规划耗时</div><span>${fmt(route.assignment?.metrics?.planningTimeMs || 0, 2)} ms</span>
            <div>路径长度</div><span>${fmt(route.assignment?.metrics?.pathLengthMeters || 0, 1)} m</span>
            <div>扩展节点</div><span>${route.assignment?.metrics?.expandedNodes || 0}</span>
            <div>样本数</div><span>${(route.telemetry || []).length}</span>
            <div>命中目标</div><span>${route.assignment?.planned ? '是' : '否'}</span>
          </div></div>
        </div>`;
      const routeGrid = document.createElement('div');
      routeGrid.className = 'chart-grid';
      metrics.forEach((metric) => {
        const series = [{
          name: route.missile?.name || `导弹-${index + 1}`,
          color: palette[index % palette.length],
          samples: (route.telemetry || []).map(sample => ({ t: sample.timeSeconds, v: sample[metric.key] }))
        }];
        routeGrid.appendChild(chartCard(metric, series, '暂无该航线数据'));
      });
      panel.appendChild(routeGrid);
      panel.insertAdjacentHTML('beforeend', `<div class="card" style="margin-top:14px;"><div class="card-header"><h3>航路点列表</h3><span class="subtle">${(route.routeWaypoints || []).length} 个点</span></div><div class="card-body">${wpTable(route.routeWaypoints || [])}</div></div>`);
      routeWrapper.appendChild(panel);
    });

    function activateRoute(index) {
      tabs.querySelectorAll('.route-tab').forEach((btn, i) => btn.classList.toggle('active', i === index));
      routeWrapper.querySelectorAll('.route-panel').forEach((panel, i) => panel.classList.toggle('active', i === index));
    }
  } else {
    routeWrapper.insertAdjacentHTML('beforeend', '<div class="empty">暂无航线数据</div>');
  }
  app.appendChild(routeWrapper);
})();
</script>
</body>
</html>)COPILOTHTML");

    return html;
}

}  // namespace mission
