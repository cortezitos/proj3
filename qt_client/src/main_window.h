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

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget * parent = nullptr);

private:
    void connect_to_server();
    void disconnect_from_server();
    void process_incoming_data();
    void apply_quaternion(const Quaternion & quaternion);

    QLineEdit * host_edit_ = nullptr;
    QSpinBox * port_spin_ = nullptr;
    QPushButton * connect_button_ = nullptr;
    QLabel * status_label_ = nullptr;
    QLabel * quaternion_label_ = nullptr;
    OrientationWidget * orientation_widget_ = nullptr;
    QTcpSocket * socket_ = nullptr;
    QByteArray buffer_;
    int sample_count_ = 0;
};
