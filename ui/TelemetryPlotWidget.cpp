#include "ui/TelemetryPlotWidget.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPen>
#include <QRect>

#include "core/MissionTypes.h"

namespace {

enum class PlotKind {
    AltitudeKm,
    RemainingKm,
    Speed,
    Pitch,
    Heading,
    Acceleration
};

struct PlotConfig {
    PlotKind kind;
    QString title;
};

struct ValueRange {
    double minValue = 0.0;
    double maxValue = 1.0;
};

const std::array<PlotConfig, 6>& plotConfigs() {
    static const std::array<PlotConfig, 6> configs = {
        PlotConfig{PlotKind::AltitudeKm, QStringLiteral("1-时间(秒)-高度(千米)")},
        PlotConfig{PlotKind::RemainingKm, QStringLiteral("2-时间(秒)-弹目距离(千米)")},
        PlotConfig{PlotKind::Speed, QStringLiteral("3-时间(秒)-速度(米/秒)")},
        PlotConfig{PlotKind::Pitch, QStringLiteral("4-时间(秒)-发射系俯仰角(度)")},
        PlotConfig{PlotKind::Heading, QStringLiteral("5-时间(秒)-发射系航向角(度)")},
        PlotConfig{PlotKind::Acceleration, QStringLiteral("6-时间(秒)-加速度(米/秒²)")}};
    return configs;
}

double valueForSample(const TelemetryPlotWidget::Sample& sample, PlotKind kind) {
    switch (kind) {
        case PlotKind::AltitudeKm:
            return sample.altitudeMeters / 1000.0;
        case PlotKind::RemainingKm:
            return sample.remainingMeters / 1000.0;
        case PlotKind::Speed:
            return sample.speedMetersPerSecond;
        case PlotKind::Pitch:
            return sample.pitchDegrees;
        case PlotKind::Heading:
            return sample.headingDegrees;
        case PlotKind::Acceleration:
        default:
            return sample.accelerationMetersPerSecond2;
    }
}

double safeRange(double minValue, double maxValue) {
    const double range = maxValue - minValue;
    return range < 1e-6 ? 1.0 : range;
}

double clampToUnit(double value) {
    return std::clamp(value, 0.0, 1.0);
}

QPointF samplePoint(
    const QRectF& plotRect,
    double tMin,
    double tMax,
    double timeValue,
    double value,
    double minValue,
    double maxValue) {
    const double timeNormalized = clampToUnit((timeValue - tMin) / safeRange(tMin, tMax));
    const double x = plotRect.left() + timeNormalized * plotRect.width();
    const double normalized = clampToUnit((value - minValue) / safeRange(minValue, maxValue));
    const double y = plotRect.bottom() - normalized * plotRect.height();
    return {x, y};
}

ValueRange computeValueRange(const std::deque<TelemetryPlotWidget::Sample>& samples, PlotKind kind) {
    ValueRange range;
    if (samples.empty()) {
        return range;
    }

    range.minValue = valueForSample(samples.front(), kind);
    range.maxValue = range.minValue;
    for (const auto& sample : samples) {
        const double value = valueForSample(sample, kind);
        range.minValue = std::min(range.minValue, value);
        range.maxValue = std::max(range.maxValue, value);
    }

    const double rangeSize = range.maxValue - range.minValue;
    if (rangeSize < 1e-5) {
        const double center = range.maxValue;
        const double padding = std::max(1.0, std::abs(center) * 0.1);
        range.minValue = center - padding;
        range.maxValue = center + padding;
        return range;
    }

    const double padding = rangeSize * 0.12;
    range.minValue -= padding;
    range.maxValue += padding;
    return range;
}

ValueRange mergeValueRanges(const std::vector<std::deque<TelemetryPlotWidget::Sample>>& allSamples, PlotKind kind) {
    ValueRange merged;
    bool first = true;
    for (const auto& samples : allSamples) {
        if (samples.empty()) continue;
        ValueRange r = computeValueRange(samples, kind);
        if (first) {
            merged = r;
            first = false;
        } else {
            merged.minValue = std::min(merged.minValue, r.minValue);
            merged.maxValue = std::max(merged.maxValue, r.maxValue);
        }
    }
    return merged;
}

void computeMergedTimeRange(
    const std::vector<std::deque<TelemetryPlotWidget::Sample>>& allSamples,
    double& tMin,
    double& tMax) {
    tMin = 0.0;
    tMax = 1.0;
    bool first = true;
    for (const auto& samples : allSamples) {
        if (samples.empty()) continue;
        if (first) {
            tMin = samples.front().timeSeconds;
            tMax = samples.back().timeSeconds;
            first = false;
        } else {
            tMin = std::min(tMin, samples.front().timeSeconds);
            tMax = std::max(tMax, samples.back().timeSeconds);
        }
    }
    if (tMax <= tMin) tMax = tMin + 1.0;
}

QString tickText(double value) {
    const double absValue = std::abs(value);
    if (absValue >= 1000.0) {
        return QString::number(value, 'f', 0);
    }
    if (absValue >= 100.0) {
        return QString::number(value, 'f', 1);
    }
    return QString::number(value, 'f', 2);
}

const TelemetryPlotWidget::Sample* findSampleAtTime(
    const std::deque<TelemetryPlotWidget::Sample>& samples,
    double targetTime) {
    if (samples.empty()) return nullptr;
    const TelemetryPlotWidget::Sample* best = &samples.front();
    double bestDist = std::abs(best->timeSeconds - targetTime);
    for (const auto& s : samples) {
        const double dist = std::abs(s.timeSeconds - targetTime);
        if (dist < bestDist) {
            bestDist = dist;
            best = &s;
        }
    }
    return best;
}

struct SubPlotResult {
    QRect outerRect;
    QRectF plotRect;
    double tMin = 0.0;
    double tMax = 1.0;
    ValueRange yRange;
    bool hasData = false;
};

SubPlotResult drawSubPlotBackground(
    QPainter& painter,
    const QRect& outerRect,
    const std::vector<std::deque<TelemetryPlotWidget::Sample>>& allSamples,
    const PlotConfig& config) {
    SubPlotResult result;
    result.outerRect = outerRect;

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(15, 28, 44));
    painter.drawRoundedRect(outerRect.adjusted(0, 0, -1, -1), 4.0, 4.0);

    painter.setPen(QColor(52, 79, 103));
    painter.setBrush(Qt::NoBrush);
    painter.drawRoundedRect(outerRect.adjusted(0, 0, -1, -1), 4.0, 4.0);

    painter.setPen(QColor(216, 233, 248));
    painter.drawText(
        outerRect.adjusted(8, 5, -6, -outerRect.height() + 22),
        Qt::AlignLeft | Qt::AlignVCenter,
        config.title);

    const QRectF plotRect = outerRect.adjusted(50, 28, -12, -32);
    result.plotRect = plotRect;

    painter.setPen(QColor(47, 72, 94));
    for (int i = 0; i <= 4; ++i) {
        const double ratio = static_cast<double>(i) / 4.0;
        const double y = plotRect.top() + ratio * plotRect.height();
        painter.drawLine(QPointF(plotRect.left(), y), QPointF(plotRect.right(), y));
    }
    for (int i = 0; i <= 4; ++i) {
        const double ratio = static_cast<double>(i) / 4.0;
        const double x = plotRect.left() + ratio * plotRect.width();
        painter.drawLine(QPointF(x, plotRect.top()), QPointF(x, plotRect.bottom()));
    }

    painter.setPen(QColor(112, 145, 173));
    painter.drawLine(QPointF(plotRect.left(), plotRect.bottom()), QPointF(plotRect.right(), plotRect.bottom()));
    painter.drawLine(QPointF(plotRect.left(), plotRect.top()), QPointF(plotRect.left(), plotRect.bottom()));

    bool anyData = false;
    for (const auto& samples : allSamples) {
        if (!samples.empty()) { anyData = true; break; }
    }

    if (!anyData) {
        painter.setPen(QColor(132, 169, 197));
        painter.drawText(plotRect.toRect(), Qt::AlignCenter, QStringLiteral("等待推演数据..."));
        result.hasData = false;
        return result;
    }

    result.hasData = true;

    computeMergedTimeRange(allSamples, result.tMin, result.tMax);
    result.yRange = mergeValueRanges(allSamples, config.kind);

    painter.setPen(QColor(176, 198, 220));
    painter.drawText(
        QRectF(plotRect.left(), plotRect.bottom() + 10.0, plotRect.width(), 16.0),
        Qt::AlignCenter,
        QStringLiteral("时间(秒)"));

    for (int i = 0; i <= 4; ++i) {
        const double ratio = static_cast<double>(i) / 4.0;
        const double yValue = result.yRange.maxValue - ratio * (result.yRange.maxValue - result.yRange.minValue);
        const double y = plotRect.top() + ratio * plotRect.height();
        const QRectF labelRect(outerRect.left() + 4.0, y - 8.0, 46.0, 16.0);
        painter.drawText(labelRect, Qt::AlignRight | Qt::AlignVCenter, tickText(yValue));
    }

    for (int i = 0; i <= 4; ++i) {
        const double ratio = static_cast<double>(i) / 4.0;
        const double x = plotRect.left() + ratio * plotRect.width();
        const double xValue = result.tMin + ratio * (result.tMax - result.tMin);
        const QRectF labelRect(x - 20.0, plotRect.bottom() + 1.0, 40.0, 14.0);
        painter.drawText(labelRect, Qt::AlignHCenter | Qt::AlignTop, QString::number(xValue, 'f', 0));
    }

    return result;
}

void drawMissileCurve(
    QPainter& painter,
    const QRectF& plotRect,
    double tMin,
    double tMax,
    const ValueRange& yRange,
    const std::deque<TelemetryPlotWidget::Sample>& samples,
    PlotKind kind,
    const QColor& color) {
    if (samples.empty()) return;

    if (samples.size() < 2) {
        const auto& latest = samples.back();
        const QPointF pt = samplePoint(
            plotRect, tMin, tMax,
            latest.timeSeconds,
            valueForSample(latest, kind),
            yRange.minValue, yRange.maxValue);
        painter.setPen(Qt::NoPen);
        painter.setBrush(color);
        painter.drawEllipse(pt, 3.0, 3.0);
    } else {
        QPainterPath path;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const auto& sample = samples[i];
            const QPointF point = samplePoint(
                plotRect, tMin, tMax,
                sample.timeSeconds,
                valueForSample(sample, kind),
                yRange.minValue, yRange.maxValue);
            if (i == 0) {
                path.moveTo(point);
            } else {
                path.lineTo(point);
            }
        }
        painter.setPen(QPen(color, 1.9));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(path);
    }
}

void drawHoverOverlay(
    QPainter& painter,
    const SubPlotResult& sp,
    const std::vector<std::deque<TelemetryPlotWidget::Sample>>& allSamples,
    const std::vector<std::string>& missileNames,
    const QPoint& hoverPos,
    PlotKind kind) {
    if (!sp.hasData) return;

    const QPointF hoverF(hoverPos);
    if (!sp.plotRect.contains(hoverF)) return;

    const double hoverRatio = (hoverF.x() - sp.plotRect.left()) / sp.plotRect.width();
    const double hoverTime = sp.tMin + clampToUnit(hoverRatio) * (sp.tMax - sp.tMin);

    painter.setPen(QPen(QColor(255, 255, 255, 80), 1.0, Qt::DashLine));
    const double lineX = sp.plotRect.left() + clampToUnit(hoverRatio) * sp.plotRect.width();
    painter.drawLine(QPointF(lineX, sp.plotRect.top()), QPointF(lineX, sp.plotRect.bottom()));

    QStringList tooltipLines;
    tooltipLines << QStringLiteral("t=%1s").arg(hoverTime, 0, 'f', 1);

    for (std::size_t m = 0; m < allSamples.size(); ++m) {
        const auto* s = findSampleAtTime(allSamples[m], hoverTime);
        if (s == nullptr) continue;
        const double val = valueForSample(*s, kind);
        const QString name = m < missileNames.size()
            ? QString::fromStdString(missileNames[m])
            : QStringLiteral("导弹-%1").arg(m + 1);
        tooltipLines << QStringLiteral("%1: %2").arg(name).arg(tickText(val));

        const QPointF pt = samplePoint(
            sp.plotRect, sp.tMin, sp.tMax,
            s->timeSeconds, val,
            sp.yRange.minValue, sp.yRange.maxValue);
        painter.setPen(Qt::NoPen);
        painter.setBrush(TelemetryPlotWidget::missileColor(static_cast<int>(m)));
        painter.drawEllipse(pt, 4.0, 4.0);
    }

    const QString tooltipText = tooltipLines.join(QStringLiteral("\n"));

    QFontMetrics fm(painter.font());
    int maxLineW = 0;
    const auto lines = tooltipText.split(QLatin1Char('\n'));
    for (const auto& line : lines) {
        maxLineW = std::max(maxLineW, fm.horizontalAdvance(line));
    }
    const int tipW = maxLineW + 16;
    const int tipH = static_cast<int>(lines.size()) * (fm.height() + 2) + 8;

    double tipX = lineX + 10.0;
    double tipY = sp.plotRect.top() + 4.0;
    if (tipX + tipW > sp.outerRect.right() - 4) {
        tipX = lineX - tipW - 10.0;
    }
    if (tipY + tipH > sp.plotRect.bottom()) {
        tipY = sp.plotRect.bottom() - tipH - 4.0;
    }

    QRectF tipRect(tipX, tipY, tipW, tipH);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(10, 22, 38, 220));
    painter.drawRoundedRect(tipRect, 4.0, 4.0);
    painter.setPen(QColor(80, 120, 160));
    painter.setBrush(Qt::NoBrush);
    painter.drawRoundedRect(tipRect, 4.0, 4.0);

    painter.setPen(QColor(220, 235, 250));
    painter.drawText(tipRect.adjusted(8, 4, -8, -4), Qt::AlignLeft | Qt::AlignTop, tooltipText);
}

}  // namespace

QColor TelemetryPlotWidget::missileColor(int index) {
    static const QColor palette[] = {
        QColor(248, 194, 79),
        QColor(119, 234, 145),
        QColor(86, 214, 255),
        QColor(236, 121, 193),
        QColor(166, 145, 245),
        QColor(255, 155, 105),
        QColor(255, 107, 107),
        QColor(78, 205, 196),
        QColor(255, 230, 109),
        QColor(168, 230, 207)};
    constexpr int count = sizeof(palette) / sizeof(palette[0]);
    return palette[std::abs(index) % count];
}

TelemetryPlotWidget::TelemetryPlotWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumHeight(330);
    setAutoFillBackground(false);
    setMouseTracking(true);
}

void TelemetryPlotWidget::clearHistory() {
    m_samples.clear();
    update();
}

void TelemetryPlotWidget::pushSample(const Sample& sample) {
    m_samples.push_back(sample);
    while (m_samples.size() > kMaxSamples) {
        m_samples.pop_front();
    }
    update();
}

void TelemetryPlotWidget::pushSample(int missileIndex, const Sample& sample) {
    if (missileIndex < 0) {
        return;
    }
    if (missileIndex >= static_cast<int>(m_missileSamples.size())) {
        m_missileSamples.resize(static_cast<std::size_t>(missileIndex + 1));
    }
    auto& deq = m_missileSamples[static_cast<std::size_t>(missileIndex)];
    deq.push_back(sample);
    while (deq.size() > kMaxSamples) {
        deq.pop_front();
    }
    update();
}

void TelemetryPlotWidget::setMissileCount(int count) {
    m_missileCount = std::max(0, count);
    if (static_cast<int>(m_missileSamples.size()) < m_missileCount) {
        m_missileSamples.resize(static_cast<std::size_t>(m_missileCount));
    }
    update();
}

void TelemetryPlotWidget::setMissileNames(const std::vector<std::string>& names) {
    m_missileNames = names;
    update();
}

void TelemetryPlotWidget::clearMissileHistory(int index) {
    if (index >= 0 && index < static_cast<int>(m_missileSamples.size())) {
        m_missileSamples[static_cast<std::size_t>(index)].clear();
    }
    update();
}

void TelemetryPlotWidget::clearAllHistory() {
    m_samples.clear();
    for (auto& deq : m_missileSamples) {
        deq.clear();
    }
    update();
}

void TelemetryPlotWidget::mouseMoveEvent(QMouseEvent* event) {
    m_hoverPos = event->pos();
    m_hovering = true;
    update();
}

void TelemetryPlotWidget::leaveEvent(QEvent* event) {
    Q_UNUSED(event);
    m_hovering = false;
    update();
}

void TelemetryPlotWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QLinearGradient bg(0.0, 0.0, 0.0, static_cast<double>(height()));
    bg.setColorAt(0.0, QColor(12, 28, 43));
    bg.setColorAt(1.0, QColor(9, 20, 32));
    painter.fillRect(rect(), bg);

    painter.setPen(QColor(38, 61, 84));
    painter.drawRoundedRect(rect().adjusted(0, 0, -1, -1), 6.0, 6.0);

    const QRect contentRect = rect().adjusted(8, 8, -8, -8);
    constexpr int rows = 2;
    constexpr int cols = 3;
    constexpr int spacingX = 8;
    constexpr int spacingY = 8;

    const int cellWidth = (contentRect.width() - spacingX * (cols - 1)) / cols;
    const int cellHeight = (contentRect.height() - spacingY * (rows - 1)) / rows;

    const auto& configs = plotConfigs();

    m_plotRects.clear();
    std::vector<SubPlotResult> subPlotResults;

    for (int index = 0; index < static_cast<int>(configs.size()); ++index) {
        const int row = index / cols;
        const int col = index % cols;
        const int x = contentRect.left() + col * (cellWidth + spacingX);
        const int y = contentRect.top() + row * (cellHeight + spacingY);
        const QRect cellRect(x, y, cellWidth, cellHeight);

        auto sp = drawSubPlotBackground(painter, cellRect, m_missileSamples, configs[static_cast<std::size_t>(index)]);
        subPlotResults.push_back(sp);
        m_plotRects.push_back(cellRect);
    }

    for (int index = 0; index < static_cast<int>(configs.size()); ++index) {
        const auto& sp = subPlotResults[index];
        if (!sp.hasData) continue;

        for (int m = 0; m < static_cast<int>(m_missileSamples.size()); ++m) {
            drawMissileCurve(
                painter, sp.plotRect, sp.tMin, sp.tMax, sp.yRange,
                m_missileSamples[m], configs[index].kind,
                missileColor(m));
        }

        if (!m_missileSamples.empty()) {
            QStringList legendParts;
            for (int m = 0; m < static_cast<int>(m_missileSamples.size()); ++m) {
                if (m_missileSamples[m].empty()) continue;
                const auto& latest = m_missileSamples[m].back();
                const double val = valueForSample(latest, configs[index].kind);
                const QString name = m < static_cast<int>(m_missileNames.size())
                    ? QString::fromStdString(m_missileNames[m])
                    : QStringLiteral("M%1").arg(m + 1);
                legendParts << QStringLiteral("%1:%2").arg(name).arg(tickText(val));
            }
            if (!legendParts.isEmpty()) {
                painter.setPen(QColor(198, 220, 240));
                painter.drawText(
                    sp.outerRect.adjusted(sp.outerRect.width() / 2, 5, -8, -sp.outerRect.height() + 22),
                    Qt::AlignRight | Qt::AlignVCenter,
                    legendParts.join(QStringLiteral(" ")));
            }
        }
    }

    if (m_hovering) {
        for (int index = 0; index < static_cast<int>(configs.size()); ++index) {
            drawHoverOverlay(
                painter, subPlotResults[index], m_missileSamples, m_missileNames,
                m_hoverPos, configs[index].kind);
        }
    }
}
