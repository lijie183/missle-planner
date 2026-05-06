#include "WaveProgressButton.h"

WaveProgressButton::WaveProgressButton(const QString& text, QWidget* parent)
    : QPushButton(text, parent)
    , m_normalText(text)
{
    setMinimumHeight(28);
    setMinimumWidth(kMinWidth);
    m_animTimer.setInterval(kAnimIntervalMs);
    connect(&m_animTimer, &QTimer::timeout, this, &WaveProgressButton::onTick);
}

void WaveProgressButton::setProgress(int percent)
{
    m_progress = qBound(0, percent, 100);
    update();
}

int WaveProgressButton::progress() const
{
    return m_progress;
}

void WaveProgressButton::startProgress(const QString& busyText)
{
    m_active = true;
    m_progress = 0;
    m_busyText = busyText.isEmpty() ? m_normalText : busyText;
    setText(m_busyText);
    setEnabled(false);
    m_phase = 0.0;
    m_elapsed.start();
    m_animTimer.start();
    update();
}

void WaveProgressButton::stopProgress()
{
    m_active = false;
    m_animTimer.stop();
    m_progress = 0;
    setText(m_normalText);
    setEnabled(true);
    update();
}

bool WaveProgressButton::isProgressActive() const
{
    return m_active;
}

QSize WaveProgressButton::minimumSizeHint() const
{
    QFontMetrics fm(font());
    return QSize(kMinWidth, fm.height() + 14);
}

QSize WaveProgressButton::sizeHint() const
{
    QFontMetrics fm(font());
    int textW = fm.horizontalAdvance(m_normalText) + 24;
    return QSize(qMax(kMinWidth, textW), fm.height() + 14);
}

void WaveProgressButton::onTick()
{
    m_phase += 0.08;
    if (m_phase > 2.0 * M_PI) {
        m_phase -= 2.0 * M_PI;
    }
    update();
}

void WaveProgressButton::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event)

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    QRectF r = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    qreal radius = 5.0;

    if (!m_active) {
        bool hovered = underMouse() && isEnabled();
        bool pressed = isDown();

        QColor bg(28, 121, 180);
        if (pressed) bg = QColor(23, 104, 155);
        else if (hovered) bg = QColor(37, 145, 212);

        QLinearGradient bgGrad(r.topLeft(), r.bottomLeft());
        bgGrad.setColorAt(0.0, bg.lighter(112));
        bgGrad.setColorAt(1.0, bg);

        p.setPen(Qt::NoPen);
        p.setBrush(bgGrad);
        p.drawRoundedRect(r, radius, radius);

        p.setPen(QColor(61, 151, 207, 180));
        p.setBrush(Qt::NoBrush);
        p.drawRoundedRect(r, radius, radius);

        p.setPen(QColor(241, 249, 255));
        p.setFont(font());
        p.drawText(r, Qt::AlignCenter, text());
        return;
    }

    QColor idleBg(16, 40, 68);
    QLinearGradient idleGrad(r.topLeft(), r.bottomLeft());
    idleGrad.setColorAt(0.0, idleBg.lighter(130));
    idleGrad.setColorAt(1.0, idleBg);
    p.setPen(Qt::NoPen);
    p.setBrush(idleGrad);
    p.drawRoundedRect(r, radius, radius);

    qreal fillRatio = m_progress / 100.0;
    qreal fillWidth = r.width() * fillRatio;

    if (fillWidth > 1.0) {
        QRectF fillRect(r.left(), r.top(), fillWidth, r.height());

        QColor energyBase(0, 200, 255);
        QLinearGradient energyGrad(r.topLeft(), r.topRight());
        qreal shift = fmod(m_phase * 0.5, 1.0);
        for (int i = 0; i <= 8; ++i) {
            qreal pos = i / 8.0;
            qreal shifted = fmod(pos + shift, 1.0);
            qreal pulse = 0.85 + 0.15 * qSin(shifted * 3.0 * M_PI);
            QColor c = energyBase.lighter(static_cast<int>(100 * pulse));
            c.setAlpha(220);
            energyGrad.setColorAt(pos, c);
        }

        QPainterPath clipPath;
        clipPath.addRoundedRect(r, radius, radius);
        p.save();
        p.setClipPath(clipPath);
        p.setPen(Qt::NoPen);
        p.setBrush(energyGrad);
        p.drawRect(fillRect);

        qreal waveX = fillRect.right();
        qreal waveAmp = r.height() * 0.08;
        QPainterPath waveEdge;
        waveEdge.moveTo(waveX, r.top());
        qreal waveStep = 1.0;
        for (qreal dy = 0; dy <= r.height(); dy += waveStep) {
            qreal xOffset = waveAmp * qSin(dy * 0.15 + m_phase * 3.0);
            waveEdge.lineTo(waveX + xOffset, r.top() + dy);
        }
        waveEdge.lineTo(waveX + waveAmp, r.bottom());
        waveEdge.lineTo(waveX + waveAmp, r.top());
        waveEdge.closeSubpath();

        QColor waveEdgeColor(100, 230, 255, 80);
        p.setBrush(waveEdgeColor);
        p.setPen(Qt::NoPen);
        p.drawPath(waveEdge);

        QRadialGradient glowGrad(QPointF(waveX, r.center().y()), r.height() * 0.8);
        glowGrad.setColorAt(0.0, QColor(160, 240, 255, 90));
        glowGrad.setColorAt(0.4, QColor(80, 200, 255, 40));
        glowGrad.setColorAt(1.0, QColor(0, 150, 255, 0));
        p.setBrush(glowGrad);
        p.setPen(Qt::NoPen);
        p.drawEllipse(QPointF(waveX, r.center().y()), r.height() * 0.8, r.height() * 0.7);

        p.restore();
    }

    qreal elapsedSec = m_elapsed.elapsed() / 1000.0;
    int rippleCount = 3;
    QPainterPath clipPath2;
    clipPath2.addRoundedRect(r, radius, radius);
    p.save();
    p.setClipPath(clipPath2);
    for (int i = 0; i < rippleCount; ++i) {
        qreal ripplePhase = fmod(elapsedSec * 0.8 + i * 0.7, 2.5);
        if (ripplePhase > 1.8) continue;
        qreal t = ripplePhase / 1.8;
        qreal rippleR = t * r.width() * 0.35;
        qreal opacity = (1.0 - t) * 0.08;

        QPointF center(r.left() + fillWidth * (0.3 + 0.4 * qSin(m_phase + i * 2.1)),
                       r.center().y() + r.height() * 0.15 * qCos(m_phase * 0.7 + i * 1.3));

        QRadialGradient rippleGrad(center, rippleR);
        QColor rc(180, 240, 255, static_cast<int>(opacity * 255));
        rippleGrad.setColorAt(0.0, rc);
        rippleGrad.setColorAt(0.6, QColor(100, 210, 255, static_cast<int>(opacity * 100)));
        rippleGrad.setColorAt(1.0, QColor(50, 170, 255, 0));
        p.setPen(Qt::NoPen);
        p.setBrush(rippleGrad);
        p.drawEllipse(center, rippleR, rippleR);
    }
    p.restore();

    p.setPen(QColor(40, 130, 200, 160));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(r, radius, radius);

    p.setPen(QColor(230, 248, 255));
    p.setFont(font());
    QString displayText = m_busyText;
    if (m_progress > 0) {
        displayText = QStringLiteral("%1 %2%").arg(m_busyText).arg(m_progress);
    }
    p.drawText(r, Qt::AlignCenter, displayText);
}
