#pragma once

#include <QPushButton>
#include <QTimer>
#include <QElapsedTimer>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>
#include <QLinearGradient>
#include <QtMath>

class WaveProgressButton : public QPushButton {
    Q_OBJECT

public:
    explicit WaveProgressButton(const QString& text, QWidget* parent = nullptr);

    void setProgress(int percent);
    int progress() const;

    void startProgress(const QString& busyText = QString());
    void stopProgress();

    bool isProgressActive() const;

    QSize minimumSizeHint() const override;
    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent* event) override;

private slots:
    void onTick();

private:
    int m_progress = 0;
    bool m_active = false;
    QString m_busyText;
    QString m_normalText;

    QTimer m_animTimer;
    QElapsedTimer m_elapsed;
    qreal m_phase = 0.0;

    static constexpr int kAnimIntervalMs = 33;
    static constexpr int kMinHeight = 36;
    static constexpr int kMinWidth = 160;
};
