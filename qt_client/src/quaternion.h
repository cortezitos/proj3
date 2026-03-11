#pragma once

#include <cmath>

struct Quaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

inline Quaternion normalizeQuaternion(const Quaternion& q) {
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0) {
        return {};
    }
    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}
