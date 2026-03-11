#pragma once

#include <QLabel>
#include <QMainWindow>

class TrackerCanvas;

class MouseTrackerWindow : public QMainWindow {
public:
    explicit MouseTrackerWindow(QWidget* parent = nullptr);

private:
    TrackerCanvas* canvas_{nullptr};
    QLabel* statusLabel_{nullptr};
};
