#include "ui/TelemetryPlotWidget.h"

#include <algorithm>
#include <cmath>

#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPen>
#include <QRect>

namespace {

constexpr std::size_t kMaxSamples = 420;

double safeRange(double minValue, double maxValue) {
    const double range = maxValue - minValue;
    return range < 1e-6 ? 1.0 : range;
}

double clampToUnit(double value) {
    return std::clamp(value, 0.0, 1.0);
}

QPointF samplePoint(
    const QRectF& plotRect,
    std::size_t index,
    std::size_t count,
    double value,
    double minValue,
    double maxValue) {
    const double t = count > 1 ? static_cast<double>(index) / static_cast<double>(count - 1) : 0.0;
    const double x = plotRect.left() + t * plotRect.width();
    const double normalized = clampToUnit((value - minValue) / safeRange(minValue, maxValue));
    const double y = plotRect.bottom() - normalized * plotRect.height();
    return {x, y};
}

void drawLegend(QPainter& painter, const QRect& rect) {
    struct LegendItem {
        QColor color;
        const char* text;
    };

    const LegendItem items[] = {
        {QColor(64, 214, 255), "speed"},
        {QColor(255, 205, 82), "altitude"},
        {QColor(221, 130, 255), "pitch"},
        {QColor(123, 242, 146), "remaining"}};

    int x = rect.left();
    const int y = rect.center().y();
    for (const auto& item : items) {
        painter.setPen(Qt::NoPen);
        painter.setBrush(item.color);
        painter.drawRoundedRect(QRect(x, y - 5, 12, 10), 2, 2);

        painter.setPen(QColor(212, 232, 250));
        painter.drawText(x + 18, y + 5, QString::fromLatin1(item.text));

        x += 112;
    }
}

}  // namespace

TelemetryPlotWidget::TelemetryPlotWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumHeight(220);
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

    painter.setPen(QColor(46, 76, 102));
    painter.drawRoundedRect(rect().adjusted(0, 0, -1, -1), 6.0, 6.0);

    const QRect legendRect = rect().adjusted(14, 8, -14, -8);
    drawLegend(painter, QRect(legendRect.left(), legendRect.top(), legendRect.width(), 18));

    const QRectF plotRect = rect().adjusted(14, 30, -14, -56);
    painter.setPen(QColor(40, 67, 90));
    for (int i = 0; i <= 4; ++i) {
        const double y = plotRect.top() + plotRect.height() * static_cast<double>(i) / 4.0;
        painter.drawLine(QPointF(plotRect.left(), y), QPointF(plotRect.right(), y));
    }
    for (int i = 0; i <= 6; ++i) {
        const double x = plotRect.left() + plotRect.width() * static_cast<double>(i) / 6.0;
        painter.drawLine(QPointF(x, plotRect.top()), QPointF(x, plotRect.bottom()));
    }

    if (m_samples.size() < 2) {
        painter.setPen(QColor(142, 182, 212));
        painter.drawText(plotRect.toRect(), Qt::AlignCenter, QStringLiteral("等待推演数据..."));
        return;
    }

    double speedMin = m_samples.front().speedMetersPerSecond;
    double speedMax = speedMin;
    double altitudeMin = m_samples.front().altitudeMeters;
    double altitudeMax = altitudeMin;
    double pitchMin = m_samples.front().pitchDegrees;
    double pitchMax = pitchMin;
    double remainMin = m_samples.front().remainingMeters;
    double remainMax = remainMin;

    for (const auto& sample : m_samples) {
        speedMin = std::min(speedMin, sample.speedMetersPerSecond);
        speedMax = std::max(speedMax, sample.speedMetersPerSecond);
        altitudeMin = std::min(altitudeMin, sample.altitudeMeters);
        altitudeMax = std::max(altitudeMax, sample.altitudeMeters);
        pitchMin = std::min(pitchMin, sample.pitchDegrees);
        pitchMax = std::max(pitchMax, sample.pitchDegrees);
        remainMin = std::min(remainMin, sample.remainingMeters);
        remainMax = std::max(remainMax, sample.remainingMeters);
    }

    QPainterPath speedPath;
    QPainterPath altitudePath;
    QPainterPath pitchPath;
    QPainterPath remainPath;

    for (std::size_t i = 0; i < m_samples.size(); ++i) {
        const auto& sample = m_samples[i];

        const QPointF pSpeed = samplePoint(plotRect, i, m_samples.size(), sample.speedMetersPerSecond, speedMin, speedMax);
        const QPointF pAlt = samplePoint(plotRect, i, m_samples.size(), sample.altitudeMeters, altitudeMin, altitudeMax);
        const QPointF pPitch = samplePoint(plotRect, i, m_samples.size(), sample.pitchDegrees, pitchMin, pitchMax);
        const QPointF pRemain = samplePoint(plotRect, i, m_samples.size(), sample.remainingMeters, remainMin, remainMax);

        if (i == 0) {
            speedPath.moveTo(pSpeed);
            altitudePath.moveTo(pAlt);
            pitchPath.moveTo(pPitch);
            remainPath.moveTo(pRemain);
        } else {
            speedPath.lineTo(pSpeed);
            altitudePath.lineTo(pAlt);
            pitchPath.lineTo(pPitch);
            remainPath.lineTo(pRemain);
        }
    }

    painter.setBrush(Qt::NoBrush);

    painter.setPen(QPen(QColor(64, 214, 255), 2.0));
    painter.drawPath(speedPath);

    painter.setPen(QPen(QColor(255, 205, 82), 1.8));
    painter.drawPath(altitudePath);

    painter.setPen(QPen(QColor(221, 130, 255), 1.6));
    painter.drawPath(pitchPath);

    painter.setPen(QPen(QColor(123, 242, 146), 1.6));
    painter.drawPath(remainPath);

    const Sample& latest = m_samples.back();
    const QRect bottomRect = rect().adjusted(14, height() - 48, -14, -10);

    painter.setPen(QColor(220, 238, 252));
    const QString text = QStringLiteral(
                             "T=%1s   V=%2m/s   ALT=%3m   pitch=%4deg   heading=%5deg   a=%6m/s2   remain=%7km   phase=%8")
                             .arg(latest.timeSeconds, 0, 'f', 1)
                             .arg(latest.speedMetersPerSecond, 0, 'f', 1)
                             .arg(latest.altitudeMeters, 0, 'f', 0)
                             .arg(latest.pitchDegrees, 0, 'f', 1)
                             .arg(latest.headingDegrees, 0, 'f', 1)
                             .arg(latest.accelerationMetersPerSecond2, 0, 'f', 2)
                             .arg(latest.remainingMeters / 1000.0, 0, 'f', 2)
                             .arg(latest.phaseText);
    painter.drawText(bottomRect, Qt::AlignLeft | Qt::AlignVCenter, text);
}
