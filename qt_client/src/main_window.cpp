#include "main_window.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QProcessEnvironment>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>

namespace
{

QString default_host()
{
    const QString host = qEnvironmentVariable("IMU_HOST");
    return host.isEmpty() ? QStringLiteral("192.168.0.179") : host;
}

int default_port()
{
    bool ok = false;
    const int port = qEnvironmentVariableIntValue("IMU_PORT", &ok);
    if (ok && (port > 0) && (port <= 65535))
    {
        return port;
    }

    return 5555;
}

} // namespace

MainWindow::MainWindow(QWidget * parent)
    : QMainWindow(parent),
      socket_(new QTcpSocket(this))
{
    auto * central = new QWidget(this);
    auto * layout = new QVBoxLayout(central);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->setSpacing(12);

    auto * heading = new QLabel("Task 3: IMU Qt Client", central);
    heading->setStyleSheet("font-size: 24px; font-weight: 700; color: #111827;");

    auto * description = new QLabel(
        "Connect to the Raspberry Pi server, read live quaternion values, and visualize the IMU orientation.",
        central);
    description->setWordWrap(true);
    description->setStyleSheet("font-size: 14px; color: #4b5563;");

    host_edit_ = new QLineEdit(default_host(), central);
    port_spin_ = new QSpinBox(central);
    port_spin_->setRange(1, 65535);
    port_spin_->setValue(default_port());

    connect_button_ = new QPushButton("Connect", central);
    status_label_ = new QLabel("Disconnected", central);
    quaternion_label_ = new QLabel("Quaternion: [1.000000 0.000000 0.000000 0.000000]", central);
    orientation_widget_ = new OrientationWidget(central);

    auto * row = new QHBoxLayout();
    row->addWidget(new QLabel("Host:", central));
    row->addWidget(host_edit_, 1);
    row->addWidget(new QLabel("Port:", central));
    row->addWidget(port_spin_);
    row->addWidget(connect_button_);

    status_label_->setStyleSheet("font-size: 15px; font-weight: 600; color: #0f172a;");
    quaternion_label_->setStyleSheet(
        "font-family: 'Courier New'; font-size: 15px; background: #f3f4f6;"
        " border-radius: 8px; padding: 10px;");

    layout->addWidget(heading);
    layout->addWidget(description);
    layout->addLayout(row);
    layout->addWidget(status_label_);
    layout->addWidget(quaternion_label_);
    layout->addWidget(orientation_widget_, 1);

    setCentralWidget(central);
    resize(860, 680);
    setWindowTitle("Project 3 Task 3 - IMU Client");

    connect(connect_button_, &QPushButton::clicked, this, [this] {
        if (socket_->state() == QAbstractSocket::ConnectedState)
        {
            disconnect_from_server();
        }
        else
        {
            connect_to_server();
        }
    });

    connect(socket_, &QTcpSocket::connected, this, [this] {
        status_label_->setText(QString("Connected to %1:%2")
                                  .arg(host_edit_->text())
                                  .arg(port_spin_->value()));
        connect_button_->setText("Disconnect");
        sample_count_ = 0;
    });

    connect(socket_, &QTcpSocket::disconnected, this, [this] {
        status_label_->setText("Disconnected");
        connect_button_->setText("Connect");
    });

    connect(socket_, &QTcpSocket::readyRead, this, [this] {
        process_incoming_data();
    });

    connect(socket_, &QTcpSocket::errorOccurred, this, [this](QAbstractSocket::SocketError) {
        status_label_->setText(QString("Socket error: %1").arg(socket_->errorString()));
        connect_button_->setText("Connect");
    });
}

void MainWindow::connect_to_server()
{
    buffer_.clear();
    socket_->abort();
    status_label_->setText("Connecting...");
    socket_->connectToHost(host_edit_->text(), static_cast<quint16>(port_spin_->value()));
}

void MainWindow::disconnect_from_server()
{
    socket_->disconnectFromHost();
    if (socket_->state() != QAbstractSocket::UnconnectedState)
    {
        socket_->waitForDisconnected(1000);
    }
}

void MainWindow::process_incoming_data()
{
    buffer_.append(socket_->readAll());

    while (true)
    {
        const int newline_index = buffer_.indexOf('\n');
        if (newline_index < 0)
        {
            break;
        }

        const QByteArray raw_line = buffer_.left(newline_index).trimmed();
        buffer_.remove(0, newline_index + 1);
        if (raw_line.isEmpty())
        {
            continue;
        }

        const QList<QByteArray> parts = raw_line.split(' ');
        if (parts.size() != 4)
        {
            continue;
        }

        bool ok = false;
        Quaternion quaternion;
        quaternion.w = parts[0].toDouble(&ok);
        if (!ok)
        {
            continue;
        }
        quaternion.x = parts[1].toDouble(&ok);
        if (!ok)
        {
            continue;
        }
        quaternion.y = parts[2].toDouble(&ok);
        if (!ok)
        {
            continue;
        }
        quaternion.z = parts[3].toDouble(&ok);
        if (!ok)
        {
            continue;
        }

        apply_quaternion(quaternion);
    }
}

void MainWindow::apply_quaternion(const Quaternion & quaternion)
{
    const Quaternion normalized = normalizeQuaternion(quaternion);
    ++sample_count_;

    quaternion_label_->setText(
        QString("Quaternion: [%1 %2 %3 %4]")
            .arg(normalized.w, 0, 'f', 6)
            .arg(normalized.x, 0, 'f', 6)
            .arg(normalized.y, 0, 'f', 6)
            .arg(normalized.z, 0, 'f', 6));

    status_label_->setText(
        QString("Receiving live data from %1:%2 | samples: %3")
            .arg(host_edit_->text())
            .arg(port_spin_->value())
            .arg(sample_count_));

    orientation_widget_->setQuaternion(normalized);
}
