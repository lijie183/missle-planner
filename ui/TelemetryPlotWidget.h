#pragma once

#include <QWidget>

#include <deque>
#include <vector>

class QComboBox;

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
    void pushSample(int missileIndex, const Sample& sample);
    void setMissileCount(int count);
    void clearMissileHistory(int index);
    void clearAllHistory();

    int selectedMissile() const;
    void setSelectedMissile(int index);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    std::deque<Sample> m_samples;
    std::vector<std::deque<Sample>> m_missileSamples;
    int m_selectedMissile = 0;
    int m_missileCount = 1;

    static constexpr std::size_t kMaxSamples = 600;
};
