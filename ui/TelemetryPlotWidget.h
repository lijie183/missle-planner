#pragma once

#include <QWidget>

#include <deque>
#include <vector>

#include "core/MissionTypes.h"

class TelemetryPlotWidget : public QWidget {
    Q_OBJECT

public:
    using Sample = mission::TelemetrySample;

    explicit TelemetryPlotWidget(QWidget* parent = nullptr);

    void clearHistory();
    void pushSample(const Sample& sample);
    void pushSample(int missileIndex, const Sample& sample);
    void setMissileCount(int count);
    void setMissileNames(const std::vector<std::string>& names);
    void clearMissileHistory(int index);
    void clearAllHistory();

    static QColor missileColor(int index);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    std::deque<Sample> m_samples;
    std::vector<std::deque<Sample>> m_missileSamples;
    std::vector<std::string> m_missileNames;
    int m_missileCount = 0;

    QPoint m_hoverPos;
    bool m_hovering = false;
    std::vector<QRect> m_plotRects;

    static constexpr std::size_t kMaxSamples = 600;
};
