#include "mouse_tracker_window.h"

#include "tracker_canvas.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

MouseTrackerWindow::MouseTrackerWindow(QWidget * parent)
    : QMainWindow(parent)
{
    auto * central = new QWidget(this);
    auto * root_layout = new QVBoxLayout(central);
    root_layout->setContentsMargins(18, 18, 18, 18);
    root_layout->setSpacing(12);

    auto * title = new QLabel("Task 2: Qt Mouse Tracker", central);
    title->setStyleSheet("font-size: 24px; font-weight: 700; color: #1f2937;");

    auto * subtitle = new QLabel(
        "Move the cursor inside the canvas to demonstrate Qt Widgets layouts, painting, and event handling.",
        central);
    subtitle->setWordWrap(true);
    subtitle->setStyleSheet("font-size: 14px; color: #475569;");

    canvas_ = new TrackerCanvas(central);
    canvas_->setStyleSheet("border: 2px solid #d4cdbf; border-radius: 10px;");

    status_label_ = new QLabel("Pointer outside canvas", central);
    status_label_->setStyleSheet("font-size: 16px; font-weight: 600; color: #0f172a;");

    auto * reset_button = new QPushButton("Clear Trail", central);
    reset_button->setStyleSheet(
        "QPushButton { background: #cf5c36; color: white; border: none;"
        " padding: 10px 16px; border-radius: 8px; font-weight: 600; }"
        "QPushButton:hover { background: #b84d2a; }");

    auto * control_row = new QHBoxLayout();
    control_row->addWidget(status_label_, 1);
    control_row->addWidget(reset_button);

    root_layout->addWidget(title);
    root_layout->addWidget(subtitle);
    root_layout->addWidget(canvas_, 1);
    root_layout->addLayout(control_row);

    setCentralWidget(central);
    resize(720, 560);
    setWindowTitle("Project 3 Task 2 - Mouse Tracker");

    connect(canvas_, &TrackerCanvas::cursorMoved, this, [this](const QPoint & point) {
        status_label_->setText(QString("Cursor: x=%1, y=%2").arg(point.x()).arg(point.y()));
    });
    connect(canvas_, &TrackerCanvas::cursorLeft, this, [this] {
        status_label_->setText("Pointer outside canvas");
    });
    connect(reset_button, &QPushButton::clicked, canvas_, &TrackerCanvas::clearTrail);
}
