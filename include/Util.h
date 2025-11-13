#pragma once
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

struct Rotation3d; // forward declaration

struct Vector2D {
    double x, y;
    Vector2D operator+(const Vector2D& o) const { return {x + o.x, y + o.y}; }
    Vector2D operator-(const Vector2D& o) const { return {x - o.x, y - o.y}; }
    Vector2D operator*(double s) const { return {x * s, y * s}; }
    Vector2D operator/(double s) const { return {x / s, y / s}; }
    double norm() const { return std::sqrt(x * x + y * y); }
};

// ============================
// Vector3D
// ============================
struct Vector3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vector3D() = default;
    Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

    double getMagnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    Vector3D normalized() const {
        double mag = getMagnitude();
        return (mag == 0) ? Vector3D{} : Vector3D{x/mag, y/mag, z/mag};
    }

    double dot(const Vector3D& o) const {
        return x*o.x + y*o.y + z*o.z;
    }

    Vector3D cross(const Vector3D& o) const {
        return Vector3D{
            y*o.z - z*o.y,
            z*o.x - x*o.z,
            x*o.y - y*o.x
        };
    }

    Vector2D toVector2D() const {
        return Vector2D{x, y};
    }

    Vector3D operator+(const Vector3D& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vector3D operator-(const Vector3D& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vector3D operator*(double s) const { return {x*s, y*s, z*s}; }
    Vector3D operator/(double s) const { return {x/s, y/s, z/s}; }
    Vector3D operator-() const { return {-x, -y, -z}; }
    Vector3D componentWiseMultiply(const Vector3D& o) const { return {x*o.x, y*o.y, z*o.z}; }

    bool operator==(const Vector3D& o) const {
        return x==o.x && y==o.y && z==o.z;
    }

    void print() const {
        std::cout << "Vector3D(" << x << ", " << y << ", " << z << ")\n";
    }
};


// ============================
// Rotation3d (Quaternion Only)
// ============================
struct Rotation3d {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Rotation3d() = default;
    Rotation3d(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // Construct from yaw/pitch/roll (radians)
    Rotation3d(double yaw, double pitch, double roll) {
        double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);

        w = cr*cp*cy + sr*sp*sy;
        x = sr*cp*cy - cr*sp*sy;
        y = cr*sp*cy + sr*cp*sy;
        z = cr*cp*sy - sr*sp*cy;
    }

    Rotation3d normalized() const {
        double mag = std::sqrt(w*w + x*x + y*y + z*z);
        return Rotation3d(w/mag, x/mag, y/mag, z/mag);
    }

    // Quaternion composition (this = this ∘ other)
    void rotateBy(const Rotation3d& o) {
        double nw = w*o.w - x*o.x - y*o.y - z*o.z;
        double nx = w*o.x + x*o.w + y*o.z - z*o.y;
        double ny = w*o.y - x*o.z + y*o.w + z*o.x;
        double nz = w*o.z + x*o.y - y*o.x + z*o.w;
        w = nw; x = nx; y = ny; z = nz;
    }

    // Convert quaternion → yaw/pitch/roll
    double getYaw() const {
        return std::atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) );
    }
    double getPitch() const {
        double s = 2*(w*y - z*x);
        return (std::abs(s) >= 1) ? std::copysign(M_PI/2, s) : std::asin(s);
    }
    double getRoll() const {
        return std::atan2( 2*(w*x + y*z), 1 - 2*(x*x + y*y) );
    }

    Vector3D getZAxis() const; // implemented in .cpp
    Vector3D getXAxis() const;
    Vector3D getYAxis() const;

    std::array<double, 3> thrustDirection() const; // implemented in .cpp
    std::array<double, 3> thrustVector(double thrustMagnitude) const;

    void print() const {
        std::cout << "Rotation3d(yaw=" << getYaw()
                  << ", pitch=" << getPitch()
                  << ", roll=" << getRoll() << ")\n";
    }

    //operators:
    

    static Rotation3d fromDegrees(double yaw, double pitch, double roll);
    static Rotation3d fromRotationMatrix(const Vector3D& xa, const Vector3D& ya, const Vector3D& za);
};


// ============================
// Pose3d
// ============================
struct Pose3d {
    Vector3D translation;
    Rotation3d rotation;

    Pose3d() = default;
    Pose3d(Vector3D t, Rotation3d r) : translation(t), rotation(r) {}
    Pose3d(double x, double y, double z, Rotation3d r) : translation(x,y,z), rotation(r) {}
    Pose3d(double x, double y, double z) : translation(x,y,z), rotation() {}

    double getX() const { return translation.x; }
    double getY() const { return translation.y; }
    double getZ() const { return translation.z; }

    Pose3d rotateBy(Rotation3d r) {
        // Rotate translation vector using quaternion
        // Convert translation to a quaternion (0, v)
        Rotation3d tQuat(0.0, translation.x, translation.y, translation.z);

        // q * v * q_conjugate
        Rotation3d q = r.normalized();
        Rotation3d qConj(q.w, -q.x, -q.y, -q.z);

        // First multiply q * tQuat
        Rotation3d temp;
        temp.w = q.w * tQuat.w - q.x * tQuat.x - q.y * tQuat.y - q.z * tQuat.z;
        temp.x = q.w * tQuat.x + q.x * tQuat.w + q.y * tQuat.z - q.z * tQuat.y;
        temp.y = q.w * tQuat.y - q.x * tQuat.z + q.y * tQuat.w + q.z * tQuat.x;
        temp.z = q.w * tQuat.z + q.x * tQuat.y - q.y * tQuat.x + q.z * tQuat.w;

        // Now multiply (q * v) * q_conjugate
        Rotation3d rotated;
        rotated.w = temp.w * qConj.w - temp.x * qConj.x - temp.y * qConj.y - temp.z * qConj.z;
        rotated.x = temp.w * qConj.x + temp.x * qConj.w + temp.y * qConj.z - temp.z * qConj.y;
        rotated.y = temp.w * qConj.y - temp.x * qConj.z + temp.y * qConj.w + temp.z * qConj.x;
        rotated.z = temp.w * qConj.z + temp.x * qConj.y - temp.y * qConj.x + temp.z * qConj.w;

        Vector3D newTranslation(rotated.x, rotated.y, rotated.z);

        // Compose orientations: new = r ∘ old
        Rotation3d newRotation = r;
        newRotation.rotateBy(rotation); // multiply r * oldRotation

        return Pose3d(newTranslation, newRotation);
    }

    void print() const {
        std::cout << "Pose3d:\n\t";
        translation.print();
        std::cout << "\t";
        rotation.print();
    }
};


// ============================
// 2D types remain unchanged
// ============================
