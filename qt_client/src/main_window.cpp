#include "main_window.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>

#include <cstdlib>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      socket_(new QTcpSocket(this)) {
    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->setSpacing(12);

    auto* heading = new QLabel("Task 3: IMU Qt Client", central);
    heading->setStyleSheet("font-size: 24px; font-weight: 700; color: #111827;");

    auto* description = new QLabel(
        "Connect to the Raspberry Pi server to read w x y z quaternion lines and visualize orientation.",
        central);
    description->setWordWrap(true);
    description->setStyleSheet("font-size: 14px; color: #4b5563;");

    hostEdit_ = new QLineEdit(QStringLiteral("192.168.0.179"), central);
    portSpin_ = new QSpinBox(central);
    portSpin_->setRange(1, 65535);
    portSpin_->setValue(5555);

    connectButton_ = new QPushButton("Connect", central);
    statusLabel_ = new QLabel("Disconnected", central);
    quaternionLabel_ = new QLabel("Quaternion: [1.000000 0.000000 0.000000 0.000000]", central);
    orientationWidget_ = new OrientationWidget(central);

    auto* row = new QHBoxLayout();
    row->addWidget(new QLabel("Host:", central));
    row->addWidget(hostEdit_, 1);
    row->addWidget(new QLabel("Port:", central));
    row->addWidget(portSpin_);
    row->addWidget(connectButton_);

    statusLabel_->setStyleSheet("font-size: 15px; font-weight: 600; color: #0f172a;");
    quaternionLabel_->setStyleSheet(
        "font-family: 'Courier New'; font-size: 15px; background: #f3f4f6;"
        " border-radius: 8px; padding: 10px;");

    layout->addWidget(heading);
    layout->addWidget(description);
    layout->addLayout(row);
    layout->addWidget(statusLabel_);
    layout->addWidget(quaternionLabel_);
    layout->addWidget(orientationWidget_, 1);

    setCentralWidget(central);
    resize(860, 680);
    setWindowTitle("Project 3 Task 3 - IMU Client");

    connect(connectButton_, &QPushButton::clicked, this, [this] {
        if (socket_->state() == QAbstractSocket::ConnectedState) {
            disconnectFromServer();
        } else {
            connectToServer();
        }
    });

    connect(socket_, &QTcpSocket::connected, this, [this] {
        statusLabel_->setText(QString("Connected to %1:%2")
                                  .arg(hostEdit_->text())
                                  .arg(portSpin_->value()));
        connectButton_->setText("Disconnect");
        sampleCount_ = 0;
    });

    connect(socket_, &QTcpSocket::disconnected, this, [this] {
        statusLabel_->setText("Disconnected");
        connectButton_->setText("Connect");
    });

    connect(socket_, &QTcpSocket::readyRead, this, [this] {
        processIncomingData();
    });

    connect(socket_, &QTcpSocket::errorOccurred, this, [this](QAbstractSocket::SocketError) {
        statusLabel_->setText(QString("Socket error: %1").arg(socket_->errorString()));
        connectButton_->setText("Connect");
    });
}

void MainWindow::connectToServer() {
    buffer_.clear();
    socket_->abort();
    statusLabel_->setText("Connecting...");
    socket_->connectToHost(hostEdit_->text(), static_cast<quint16>(portSpin_->value()));
}

void MainWindow::disconnectFromServer() {
    socket_->disconnectFromHost();
    if (socket_->state() != QAbstractSocket::UnconnectedState) {
        socket_->waitForDisconnected(1000);
    }
}

void MainWindow::processIncomingData() {
    buffer_.append(socket_->readAll());

    while (true) {
        const int newlineIndex = buffer_.indexOf('\n');
        if (newlineIndex < 0) {
            break;
        }

        const QByteArray rawLine = buffer_.left(newlineIndex).trimmed();
        buffer_.remove(0, newlineIndex + 1);
        if (rawLine.isEmpty()) {
            continue;
        }

        const QList<QByteArray> parts = rawLine.split(' ');
        if (parts.size() != 4) {
            continue;
        }

        bool ok = false;
        Quaternion quaternion;
        quaternion.w = parts[0].toDouble(&ok);
        if (!ok) {
            continue;
        }
        quaternion.x = parts[1].toDouble(&ok);
        if (!ok) {
            continue;
        }
        quaternion.y = parts[2].toDouble(&ok);
        if (!ok) {
            continue;
        }
        quaternion.z = parts[3].toDouble(&ok);
        if (!ok) {
            continue;
        }

        applyQuaternion(quaternion);
    }
}

void MainWindow::applyQuaternion(const Quaternion& quaternion) {
    const Quaternion normalized = normalizeQuaternion(quaternion);
    ++sampleCount_;

    quaternionLabel_->setText(
        QString("Quaternion: [%1 %2 %3 %4]")
            .arg(normalized.w, 0, 'f', 6)
            .arg(normalized.x, 0, 'f', 6)
            .arg(normalized.y, 0, 'f', 6)
            .arg(normalized.z, 0, 'f', 6));

    statusLabel_->setText(
        QString("Receiving live data from %1:%2 | samples: %3")
            .arg(hostEdit_->text())
            .arg(portSpin_->value())
            .arg(sampleCount_));

    orientationWidget_->setQuaternion(normalized);
}
