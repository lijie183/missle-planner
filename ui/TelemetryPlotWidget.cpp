#include "ui/TelemetryPlotWidget.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPen>
#include <QRect>
#include <QVBoxLayout>

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
    QString yAxisText;
    QColor lineColor;
};

struct ValueRange {
    double minValue = 0.0;
    double maxValue = 1.0;
};

const std::array<PlotConfig, 6>& plotConfigs() {
    static const std::array<PlotConfig, 6> configs = {
        PlotConfig{PlotKind::AltitudeKm, QStringLiteral("1-时间(秒)-高度(千米)"), QStringLiteral("高度(千米)"), QColor(248, 194, 79)},
        PlotConfig{PlotKind::RemainingKm, QStringLiteral("2-时间(秒)-弹目距离(千米)"), QStringLiteral("距离(千米)"), QColor(119, 234, 145)},
        PlotConfig{PlotKind::Speed, QStringLiteral("3-时间(秒)-速度(米/秒)"), QStringLiteral("速度(米/秒)"), QColor(86, 214, 255)},
        PlotConfig{PlotKind::Pitch, QStringLiteral("4-时间(秒)-发射系俯仰角(度)"), QStringLiteral("俯仰角(度)"), QColor(236, 121, 193)},
        PlotConfig{PlotKind::Heading, QStringLiteral("5-时间(秒)-发射系航向角(度)"), QStringLiteral("航向角(度)"), QColor(166, 145, 245)},
        PlotConfig{PlotKind::Acceleration, QStringLiteral("6-时间(秒)-加速度(米/秒²)"), QStringLiteral("加速度(米/秒²)"), QColor(255, 155, 105)}};
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

void drawSubPlot(
    QPainter& painter,
    const QRect& outerRect,
    const std::deque<TelemetryPlotWidget::Sample>& samples,
    const PlotConfig& config) {
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

    const QRectF plotRect = outerRect.adjusted(54, 28, -12, -32);

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

    if (samples.empty()) {
        painter.setPen(QColor(132, 169, 197));
        painter.drawText(plotRect.toRect(), Qt::AlignCenter, QStringLiteral("等待推演数据..."));
        return;
    }

    const double tMin = samples.front().timeSeconds;
    const double tMax = std::max(samples.back().timeSeconds, tMin + 1.0);
    const ValueRange yRange = computeValueRange(samples, config.kind);

    painter.setPen(QColor(176, 198, 220));
    painter.drawText(
        outerRect.adjusted(6, 28, -outerRect.width() + 48, -12),
        Qt::AlignTop | Qt::AlignLeft,
        config.yAxisText);
    painter.drawText(
        QRectF(plotRect.left(), plotRect.bottom() + 10.0, plotRect.width(), 16.0),
        Qt::AlignCenter,
        QStringLiteral("时间(秒)"));

    for (int i = 0; i <= 4; ++i) {
        const double ratio = static_cast<double>(i) / 4.0;
        const double yValue = yRange.maxValue - ratio * (yRange.maxValue - yRange.minValue);
        const double y = plotRect.top() + ratio * plotRect.height();
        const QRectF labelRect(outerRect.left() + 4.0, y - 8.0, 46.0, 16.0);
        painter.drawText(labelRect, Qt::AlignRight | Qt::AlignVCenter, tickText(yValue));
    }

    for (int i = 0; i <= 4; ++i) {
        const double ratio = static_cast<double>(i) / 4.0;
        const double x = plotRect.left() + ratio * plotRect.width();
        const double xValue = tMin + ratio * (tMax - tMin);
        const QRectF labelRect(x - 20.0, plotRect.bottom() + 1.0, 40.0, 14.0);
        painter.drawText(labelRect, Qt::AlignHCenter | Qt::AlignTop, QString::number(xValue, 'f', 0));
    }

    if (samples.size() < 2) {
        const auto& latest = samples.back();
        const QPointF pt = samplePoint(
            plotRect, tMin, tMax,
            latest.timeSeconds,
            valueForSample(latest, config.kind),
            yRange.minValue, yRange.maxValue);
        painter.setPen(Qt::NoPen);
        painter.setBrush(config.lineColor);
        painter.drawEllipse(pt, 3.0, 3.0);
    } else {
        QPainterPath path;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const auto& sample = samples[i];
            const QPointF point = samplePoint(
                plotRect, tMin, tMax,
                sample.timeSeconds,
                valueForSample(sample, config.kind),
                yRange.minValue, yRange.maxValue);
            if (i == 0) {
                path.moveTo(point);
            } else {
                path.lineTo(point);
            }
        }

        painter.setPen(QPen(config.lineColor, 1.9));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(path);
    }

    const auto& latest = samples.back();
    const QString latestText = QStringLiteral("当前: %1").arg(tickText(valueForSample(latest, config.kind)));
    painter.setPen(QColor(198, 220, 240));
    painter.drawText(
        outerRect.adjusted(outerRect.width() / 2, 5, -8, -outerRect.height() + 22),
        Qt::AlignRight | Qt::AlignVCenter,
        latestText);
}

}  // namespace

TelemetryPlotWidget::TelemetryPlotWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumHeight(330);
    setAutoFillBackground(false);
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
    pushSample(m_selectedMissile, sample);
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
    if (missileIndex == m_selectedMissile) {
        update();
    }
}

void TelemetryPlotWidget::setMissileCount(int count) {
    m_missileCount = std::max(1, count);
    if (static_cast<int>(m_missileSamples.size()) < m_missileCount) {
        m_missileSamples.resize(static_cast<std::size_t>(m_missileCount));
    }
    if (m_selectedMissile >= m_missileCount) {
        m_selectedMissile = 0;
    }
    update();
}

void TelemetryPlotWidget::clearMissileHistory(int index) {
    if (index >= 0 && index < static_cast<int>(m_missileSamples.size())) {
        m_missileSamples[static_cast<std::size_t>(index)].clear();
    }
    if (index == m_selectedMissile) {
        update();
    }
}

void TelemetryPlotWidget::clearAllHistory() {
    m_samples.clear();
    for (auto& deq : m_missileSamples) {
        deq.clear();
    }
    update();
}

int TelemetryPlotWidget::selectedMissile() const {
    return m_selectedMissile;
}

void TelemetryPlotWidget::setSelectedMissile(int index) {
    if (index >= 0 && index < m_missileCount) {
        m_selectedMissile = index;
        update();
    }
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

    const std::deque<Sample>* displaySamples = &m_samples;
    if (m_selectedMissile >= 0 && m_selectedMissile < static_cast<int>(m_missileSamples.size())) {
        const auto& missileData = m_missileSamples[static_cast<std::size_t>(m_selectedMissile)];
        if (!missileData.empty()) {
            displaySamples = &missileData;
        }
    }

    for (int index = 0; index < static_cast<int>(configs.size()); ++index) {
        const int row = index / cols;
        const int col = index % cols;
        const int x = contentRect.left() + col * (cellWidth + spacingX);
        const int y = contentRect.top() + row * (cellHeight + spacingY);
        const QRect cellRect(x, y, cellWidth, cellHeight);
        drawSubPlot(painter, cellRect, *displaySamples, configs[static_cast<std::size_t>(index)]);
    }
}
