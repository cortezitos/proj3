#pragma once

#include "orientation_widget.h"
#include "quaternion.h"

#include <QByteArray>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QTcpSocket>

class QSpinBox;
class QPushButton;

class MainWindow : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);

private:
    void connectToServer();
    void disconnectFromServer();
    void processIncomingData();
    void applyQuaternion(const Quaternion& quaternion);

    QLineEdit* hostEdit_{nullptr};
    QSpinBox* portSpin_{nullptr};
    QPushButton* connectButton_{nullptr};
    QLabel* statusLabel_{nullptr};
    QLabel* quaternionLabel_{nullptr};
    OrientationWidget* orientationWidget_{nullptr};
    QTcpSocket* socket_{nullptr};
    QByteArray buffer_;
    int sampleCount_{0};
};
