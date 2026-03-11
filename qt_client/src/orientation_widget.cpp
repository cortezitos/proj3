#include "orientation_widget.h"

#include <QPainter>
#include <QPaintEvent>
#include <QPainterPath>
#include <array>

namespace {

struct Vec3 {
    double x;
    double y;
    double z;
};

Vec3 rotatePoint(const Quaternion& qIn, const Vec3& v) {
    const Quaternion q = normalizeQuaternion(qIn);

    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    const double xy = q.x * q.y;
    const double xz = q.x * q.z;
    const double yz = q.y * q.z;
    const double wx = q.w * q.x;
    const double wy = q.w * q.y;
    const double wz = q.w * q.z;

    return {
        (1.0 - 2.0 * (yy + zz)) * v.x + 2.0 * (xy - wz) * v.y + 2.0 * (xz + wy) * v.z,
        2.0 * (xy + wz) * v.x + (1.0 - 2.0 * (xx + zz)) * v.y + 2.0 * (yz - wx) * v.z,
        2.0 * (xz - wy) * v.x + 2.0 * (yz + wx) * v.y + (1.0 - 2.0 * (xx + yy)) * v.z,
    };
}

QPointF projectPoint(const Vec3& point, const QRectF& viewport) {
    const double cameraDistance = 4.5;
    const double depth = cameraDistance - point.z;
    const double scale = std::min(viewport.width(), viewport.height()) * 0.24;
    const double factor = scale / depth;
    return {
        viewport.center().x() + point.x * factor,
        viewport.center().y() - point.y * factor,
    };
}

}  // namespace

OrientationWidget::OrientationWidget(QWidget* parent)
    : QWidget(parent) {
    setMinimumSize(480, 380);
}

void OrientationWidget::setQuaternion(const Quaternion& quaternion) {
    quaternion_ = normalizeQuaternion(quaternion);
    update();
}

void OrientationWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    QLinearGradient background(rect().topLeft(), rect().bottomRight());
    background.setColorAt(0.0, QColor("#102542"));
    background.setColorAt(1.0, QColor("#18314f"));
    painter.fillRect(rect(), background);

    const QRectF viewport = rect().adjusted(24, 24, -24, -24);

    painter.setPen(QPen(QColor(255, 255, 255, 35), 1));
    painter.drawEllipse(viewport.center(), viewport.width() * 0.32, viewport.width() * 0.32);

    const std::array<Vec3, 8> cube = {{{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                                       {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}}};
    const std::array<std::pair<int, int>, 12> edges = {{{0, 1}, {1, 2}, {2, 3}, {3, 0},
                                                         {4, 5}, {5, 6}, {6, 7}, {7, 4},
                                                         {0, 4}, {1, 5}, {2, 6}, {3, 7}}};

    std::array<QPointF, 8> projected{};
    for (int i = 0; i < static_cast<int>(cube.size()); ++i) {
        projected[i] = projectPoint(rotatePoint(quaternion_, cube[i]), viewport);
    }

    painter.setPen(QPen(QColor("#f4f1de"), 2));
    for (const auto& edge : edges) {
        painter.drawLine(projected[edge.first], projected[edge.second]);
    }

    painter.setPen(QPen(QColor("#ef476f"), 3));
    painter.drawLine(viewport.center(), projectPoint(rotatePoint(quaternion_, {1.6, 0, 0}), viewport));
    painter.setPen(QPen(QColor("#06d6a0"), 3));
    painter.drawLine(viewport.center(), projectPoint(rotatePoint(quaternion_, {0, 1.6, 0}), viewport));
    painter.setPen(QPen(QColor("#118ab2"), 3));
    painter.drawLine(viewport.center(), projectPoint(rotatePoint(quaternion_, {0, 0, 1.6}), viewport));

    painter.setPen(QColor("#f8fafc"));
    painter.setFont(QFont("Sans Serif", 11, QFont::Bold));
    painter.drawText(QRectF(24, 24, width() - 48, 28), Qt::AlignLeft | Qt::AlignVCenter,
                     QString("Orientation view from quaternion [%1, %2, %3, %4]")
                         .arg(quaternion_.w, 0, 'f', 3)
                         .arg(quaternion_.x, 0, 'f', 3)
                         .arg(quaternion_.y, 0, 'f', 3)
                         .arg(quaternion_.z, 0, 'f', 3));
}
