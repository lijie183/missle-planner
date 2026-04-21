#pragma once

#include <QWidget>

#include <deque>

class TelemetryPlotWidget : public QWidget {
    Q_OBJECT

public:
    struct Sample {
        double timeSeconds = 0.0;
        double speedMetersPerSecond = 0.0;
        double altitudeMeters = 0.0;
        double pitchDegrees = 0.0;
        double headingDegrees = 0.0;
        double remainingMeters = 0.0;
        double accelerationMetersPerSecond2 = 0.0;
    };

    explicit TelemetryPlotWidget(QWidget* parent = nullptr);

    void clearHistory();
    void pushSample(const Sample& sample);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    std::deque<Sample> m_samples;
};
