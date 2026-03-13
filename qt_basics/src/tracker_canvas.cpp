#include "tracker_canvas.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>

TrackerCanvas::TrackerCanvas(QWidget * parent)
    : QWidget(parent)
{
    setMouseTracking(true);
    setMinimumSize(420, 320);
}

void TrackerCanvas::clearTrail()
{
    trail_.clear();
    has_point_ = false;
    update();
}

void TrackerCanvas::mouseMoveEvent(QMouseEvent * event)
{
    last_point_ = event->pos();
    has_point_ = true;
    trail_.push_back(last_point_);
    if (trail_.size() > 80)
    {
        trail_.remove(0, trail_.size() - 80);
    }
    emit cursorMoved(last_point_);
    update();
}

void TrackerCanvas::leaveEvent(QEvent * event)
{
    Q_UNUSED(event);
    has_point_ = false;
    emit cursorLeft();
    update();
}

void TrackerCanvas::paintEvent(QPaintEvent * event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor("#fcfaf4"));

    painter.setPen(QPen(QColor("#d4cdbf"), 1));
    for (int x = 0; x < width(); x += 32)
    {
        painter.drawLine(x, 0, x, height());
    }
    for (int y = 0; y < height(); y += 32)
    {
        painter.drawLine(0, y, width(), y);
    }

    if (!trail_.isEmpty())
    {
        painter.setPen(QPen(QColor("#cf5c36"), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        for (int i = 1; i < trail_.size(); ++i)
        {
            painter.drawLine(trail_[i - 1], trail_[i]);
        }
    }

    if (has_point_)
    {
        painter.setPen(QPen(QColor("#1f3c88"), 1, Qt::DashLine));
        painter.drawLine(last_point_.x(), 0, last_point_.x(), height());
        painter.drawLine(0, last_point_.y(), width(), last_point_.y());

        painter.setBrush(QColor("#1f3c88"));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(last_point_, 6, 6);
    }
}
