#pragma once
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

struct Quaternion; // forward declaration

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
struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Quaternion() = default;
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // Construct from yaw/pitch/roll (radians)
    Quaternion(double yaw, double pitch, double roll) {
        double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);

        w = cr*cp*cy + sr*sp*sy;
        x = sr*cp*cy - cr*sp*sy;
        y = cr*sp*cy + sr*cp*sy;
        z = cr*cp*sy - sr*sp*cy;
    }

    Quaternion normalized() const {
        double mag = std::sqrt(w*w + x*x + y*y + z*z);
        return Quaternion(w/mag, x/mag, y/mag, z/mag);
    }

    // Quaternion composition (this = this ∘ other)
    void rotateBy(const Quaternion& o) {
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
    Vector3D getRotationVector() {
        return Vector3D(getRoll(), getPitch(), getYaw());
    }

    std::array<double, 3> thrustDirection() const; // implemented in .cpp
    std::array<double, 3> thrustVector(double thrustMagnitude) const;

    Quaternion inverse() const;

    void print() const {
        std::cout << "Rotation3d(yaw=" << getYaw()
                  << ", pitch=" << getPitch()
                  << ", roll=" << getRoll() << ")\n";
    }

    //operators:
    

    static Quaternion fromDegrees(double yaw, double pitch, double roll);
    static Quaternion fromRotationMatrix(const Vector3D& xa, const Vector3D& ya, const Vector3D& za);
};


// ============================
// Pose3d
// ============================
struct Pose3D {
    Vector3D translation;
    Quaternion rotation;

    Pose3D() = default;
    Pose3D(Vector3D t, Quaternion r) : translation(t), rotation(r) {}
    Pose3D(double x, double y, double z, Quaternion r) : translation(x,y,z), rotation(r) {}
    Pose3D(double x, double y, double z) : translation(x,y,z), rotation() {}

    double getX() const { return translation.x; }
    double getY() const { return translation.y; }
    double getZ() const { return translation.z; }

    Pose3D rotateBy(Quaternion r) {
        // Rotate translation vector using quaternion
        // Convert translation to a quaternion (0, v)
        Quaternion tQuat(0.0, translation.x, translation.y, translation.z);

        // q * v * q_conjugate
        Quaternion q = r.normalized();
        Quaternion qConj(q.w, -q.x, -q.y, -q.z);

        // First multiply q * tQuat
        Quaternion temp;
        temp.w = q.w * tQuat.w - q.x * tQuat.x - q.y * tQuat.y - q.z * tQuat.z;
        temp.x = q.w * tQuat.x + q.x * tQuat.w + q.y * tQuat.z - q.z * tQuat.y;
        temp.y = q.w * tQuat.y - q.x * tQuat.z + q.y * tQuat.w + q.z * tQuat.x;
        temp.z = q.w * tQuat.z + q.x * tQuat.y - q.y * tQuat.x + q.z * tQuat.w;

        // Now multiply (q * v) * q_conjugate
        Quaternion rotated;
        rotated.w = temp.w * qConj.w - temp.x * qConj.x - temp.y * qConj.y - temp.z * qConj.z;
        rotated.x = temp.w * qConj.x + temp.x * qConj.w + temp.y * qConj.z - temp.z * qConj.y;
        rotated.y = temp.w * qConj.y - temp.x * qConj.z + temp.y * qConj.w + temp.z * qConj.x;
        rotated.z = temp.w * qConj.z + temp.x * qConj.y - temp.y * qConj.x + temp.z * qConj.w;

        Vector3D newTranslation(rotated.x, rotated.y, rotated.z);

        // Compose orientations: new = r ∘ old
        Quaternion newRotation = r;
        newRotation.rotateBy(rotation); // multiply r * oldRotation

        return Pose3D(newTranslation, newRotation);
    }

    void print() const {
        std::cout << "Pose3d:\n\t";
        translation.print();
        std::cout << "\t";
        rotation.print();
    }
};

// struct Twist3D {
//     Vector3D linear, angular;

//     Twist3D(Vector3D linear, Vector3D angular): linear(linear), angular(angular) {
//     }

//     Twist3D(Vector3D linear, double yaw, double pitch, double roll): linear(linear), angular(Vector3D(roll, pitch, yaw)){

//     }

//     double getYaw() {
//         return angular.z;
//     }

//     double getPitch() {
//         return angular.y;
//     }

//     double getRoll() {
//         return angular.x;
//     }

//     Pose3D apply(Pose3D other) {
//         return Pose3D(
//             (other.translation + linear),
//             other.rotation.rotateBy(Quaternion(angular.z, angular.y, angular.x))
//         );
//     }
// };