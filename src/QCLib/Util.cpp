// Util.cpp
#include "Util.h"
#include <cmath>
#include <array>
#include <vector>
#include <iostream>
#include <stdexcept> // For std::runtime_error

// -----------------------------
// Utility functions
// -----------------------------
double interpolateLinear(const std::vector<double>& x_data,
                         const std::vector<double>& y_data,
                         double x_interp) {
    if (x_data.size() != y_data.size() || x_data.empty()) {
        throw std::runtime_error("Input arrays must have the same non-zero size.");
    }

    if (x_interp <= x_data.front()) return y_data.front();
    if (x_interp >= x_data.back())  return y_data.back();

    for (size_t i = 0; i < x_data.size() - 1; ++i) {
        if (x_interp >= x_data[i] && x_interp <= x_data[i+1]) {
            double x1 = x_data[i];
            double y1 = y_data[i];
            double x2 = x_data[i+1];
            double y2 = y_data[i+1];
            return y1 + (x_interp - x1) * (y2 - y1) / (x2 - x1);
        }
    }

    throw std::runtime_error("Could not find interpolation interval.");
}

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}


// -----------------------------
// Vector3d
// -----------------------------


// -----------------------------
// Quaternion / rotation helper
// -----------------------------
// Rotate vector v by quaternion q (this quaternion). Uses formula:
// v' = v + 2*cross(q_vec, cross(q_vec, v) + w*v)
static Vector3D rotateVectorByQuat(const Quaternion& q, const Vector3D& v) {
    // q = (w, qv)
    Vector3D qv{ q.x, q.y, q.z };
    // t = 2 * cross(qv, v)
    Vector3D t = qv.cross(v) * 2.0;
    // v' = v + w * t + cross(qv, t)
    Vector3D res = v + (t * q.w) + qv.cross(t);
    return res;
}


// -----------------------------
// Rotation3d implementations
// -----------------------------
Quaternion Quaternion::fromRotationMatrix(const Vector3D& x_axis, const Vector3D& y_axis, const Vector3D& z_axis) {
    // Interpret columns as x_axis, y_axis, z_axis (world coords of body axes)
    double m00 = x_axis.x;
    double m01 = y_axis.x;
    double m02 = z_axis.x;
    double m10 = x_axis.y;
    double m11 = y_axis.y;
    double m12 = z_axis.y;
    double m20 = x_axis.z;
    double m21 = y_axis.z;
    double m22 = z_axis.z;

    double trace = m00 + m11 + m22;
    double qw, qx, qy, qz;

    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (m21 - m12) * s;
        qy = (m02 - m20) * s;
        qz = (m10 - m01) * s;
    } else {
        if (m00 > m11 && m00 > m22) {
            double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
            qw = (m21 - m12) / s;
            qx = 0.25 * s;
            qy = (m01 + m10) / s;
            qz = (m02 + m20) / s;
        } else if (m11 > m22) {
            double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
            qw = (m02 - m20) / s;
            qx = (m01 + m10) / s;
            qy = 0.25 * s;
            qz = (m12 + m21) / s;
        } else {
            double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
            qw = (m10 - m01) / s;
            qx = (m02 + m20) / s;
            qy = (m12 + m21) / s;
            qz = 0.25 * s;
        }
    }

    return Quaternion(qw, qx, qy, qz);
}

Quaternion Quaternion::fromDegrees(double yaw, double pitch, double roll) {
    return Quaternion(degreesToRadians(yaw), degreesToRadians(pitch), degreesToRadians(roll));
}

Vector3D Quaternion::getZAxis() const {
    // Body Z axis in world coordinates = rotate (0,0,1) by quaternion
    Vector3D bodyZ{0.0, 0.0, 1.0};
    Vector3D worldZ = rotateVectorByQuat(*this, bodyZ);
    return worldZ.normalized();
}

Vector3D Quaternion::getXAxis() const {
    Vector3D bodyX{1.0, 0.0, 0.0};
    Vector3D worldX = rotateVectorByQuat(*this, bodyX);
    return worldX.normalized();
}

Vector3D Quaternion::getYAxis() const {
    Vector3D bodyY{0.0, 1.0, 0.0};
    Vector3D worldY = rotateVectorByQuat(*this, bodyY);
    return worldY.normalized();
}

std::array<double, 3> Quaternion::thrustDirection() const {
    // Thrust acts along body -Z. Compute world direction of body -Z.
    Vector3D negBodyZ = rotateVectorByQuat(*this, Vector3D{0.0, 0.0, 1.0});
    double mag = negBodyZ.getMagnitude();
    if (mag == 0.0) return {0.0, 0.0, 0.0};
    return {negBodyZ.x / mag, negBodyZ.y / mag, negBodyZ.z / mag};
}

std::array<double, 3> Quaternion::thrustVector(double thrustMagnitude) const {
    auto dir = thrustDirection();
    return { dir[0] * thrustMagnitude, dir[1] * thrustMagnitude, dir[2] * thrustMagnitude };
}

Quaternion Quaternion::inverse() const {
    // Compute magnitude (norm)
    double norm = std::sqrt(w*w + x*x + y*y + z*z);

    // Guard against divide-by-zero
    if (norm == 0.0) {
        // Return identity quaternion if something is horribly wrong
        return Quaternion(1.0, 0.0, 0.0, 0.0);
    }

    double invNorm = 1.0 / norm;

    // For rotation quaternions, inverse = conjugate / |q|
    return Quaternion(
        w * invNorm,
        -x * invNorm,
        -y * invNorm,
        -z * invNorm
    );
}

