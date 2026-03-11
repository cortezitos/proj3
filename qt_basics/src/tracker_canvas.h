#pragma once

#include <QPoint>
#include <QVector>
#include <QWidget>

class TrackerCanvas : public QWidget {
    Q_OBJECT

public:
    explicit TrackerCanvas(QWidget* parent = nullptr);

    void clearTrail();

signals:
    void cursorMoved(const QPoint& position);
    void cursorLeft();

protected:
    void mouseMoveEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

private:
    QVector<QPoint> trail_;
    QPoint lastPoint_;
    bool hasPoint_{false};
};
