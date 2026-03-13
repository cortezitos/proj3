#pragma once

#include "quaternion.h"

#include <QWidget>

class OrientationWidget : public QWidget
{
public:
    explicit OrientationWidget(QWidget * parent = nullptr);

    void setQuaternion(const Quaternion & quaternion);

protected:
    void paintEvent(QPaintEvent * event) override;

private:
    Quaternion quaternion_{};
};
