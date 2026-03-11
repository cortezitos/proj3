#include "mouse_tracker_window.h"

#include "tracker_canvas.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

MouseTrackerWindow::MouseTrackerWindow(QWidget* parent)
    : QMainWindow(parent) {
    auto* central = new QWidget(this);
    auto* rootLayout = new QVBoxLayout(central);
    rootLayout->setContentsMargins(18, 18, 18, 18);
    rootLayout->setSpacing(12);

    auto* title = new QLabel("Task 2: Qt Mouse Tracker", central);
    title->setStyleSheet("font-size: 24px; font-weight: 700; color: #1f2937;");

    auto* subtitle = new QLabel(
        "Move the cursor inside the canvas to prove basic Qt Widgets event handling.",
        central);
    subtitle->setWordWrap(true);
    subtitle->setStyleSheet("font-size: 14px; color: #475569;");

    canvas_ = new TrackerCanvas(central);
    canvas_->setStyleSheet("border: 2px solid #d4cdbf; border-radius: 10px;");

    statusLabel_ = new QLabel("Pointer outside canvas", central);
    statusLabel_->setStyleSheet("font-size: 16px; font-weight: 600; color: #0f172a;");

    auto* resetButton = new QPushButton("Clear Trail", central);
    resetButton->setStyleSheet(
        "QPushButton { background: #cf5c36; color: white; border: none;"
        " padding: 10px 16px; border-radius: 8px; font-weight: 600; }"
        "QPushButton:hover { background: #b84d2a; }");

    auto* controlRow = new QHBoxLayout();
    controlRow->addWidget(statusLabel_, 1);
    controlRow->addWidget(resetButton);

    rootLayout->addWidget(title);
    rootLayout->addWidget(subtitle);
    rootLayout->addWidget(canvas_, 1);
    rootLayout->addLayout(controlRow);

    setCentralWidget(central);
    resize(720, 560);
    setWindowTitle("Project 3 Task 2 - Mouse Tracker");

    connect(canvas_, &TrackerCanvas::cursorMoved, this, [this](const QPoint& point) {
        statusLabel_->setText(QString("Cursor: x=%1, y=%2").arg(point.x()).arg(point.y()));
    });
    connect(canvas_, &TrackerCanvas::cursorLeft, this, [this] {
        statusLabel_->setText("Pointer outside canvas");
    });
    connect(resetButton, &QPushButton::clicked, canvas_, &TrackerCanvas::clearTrail);
}
