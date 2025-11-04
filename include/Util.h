#pragma once
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

struct Rotation3d; // forward declaration


// ============================
// Vector3d
// ============================
struct Vector3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vector3d() = default;
    Vector3d(double x, double y, double z) : x(x), y(y), z(z) {}

    double getMagnitude() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    Vector3d normalized() const {
        double mag = getMagnitude();
        return (mag == 0) ? Vector3d{} : Vector3d{x/mag, y/mag, z/mag};
    }

    double dot(const Vector3d& o) const {
        return x*o.x + y*o.y + z*o.z;
    }

    Vector3d cross(const Vector3d& o) const {
        return Vector3d{
            y*o.z - z*o.y,
            z*o.x - x*o.z,
            x*o.y - y*o.x
        };
    }

    Vector3d operator+(const Vector3d& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vector3d operator-(const Vector3d& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vector3d operator*(double s) const { return {x*s, y*s, z*s}; }
    Vector3d operator/(double s) const { return {x/s, y/s, z/s}; }
    Vector3d operator-() const { return {-x, -y, -z}; }
    Vector3d componentWiseMultiply(const Vector3d& o) const { return {x*o.x, y*o.y, z*o.z}; }

    bool operator==(const Vector3d& o) const {
        return x==o.x && y==o.y && z==o.z;
    }

    void print() const {
        std::cout << "Vector3d(" << x << ", " << y << ", " << z << ")\n";
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

    Vector3d getZAxis() const; // implemented in .cpp
    Vector3d getXAxis() const;
    Vector3d getYAxis() const;

    std::array<double, 3> thrustDirection() const; // implemented in .cpp
    std::array<double, 3> thrustVector(double thrustMagnitude) const;

    void print() const {
        std::cout << "Rotation3d(yaw=" << getYaw()
                  << ", pitch=" << getPitch()
                  << ", roll=" << getRoll() << ")\n";
    }

    static Rotation3d fromDegrees(double yaw, double pitch, double roll);
    static Rotation3d fromRotationMatrix(const Vector3d& xa, const Vector3d& ya, const Vector3d& za);
};


// ============================
// Pose3d
// ============================
struct Pose3d {
    Vector3d translation;
    Rotation3d rotation;

    Pose3d() = default;
    Pose3d(Vector3d t, Rotation3d r) : translation(t), rotation(r) {}
    Pose3d(double x, double y, double z, Rotation3d r) : translation(x,y,z), rotation(r) {}
    Pose3d(double x, double y, double z) : translation(x,y,z), rotation() {}

    double getX() const { return translation.x; }
    double getY() const { return translation.y; }
    double getZ() const { return translation.z; }

    void print() const {
        std::cout << "Pose3d:\n  ";
        translation.print();
        rotation.print();
    }
};


// ============================
// 2D types remain unchanged
// ============================
struct Vector2D {
    double x, y;
    Vector2D operator+(const Vector2D& o) const { return {x + o.x, y + o.y}; }
    Vector2D operator-(const Vector2D& o) const { return {x - o.x, y - o.y}; }
    Vector2D operator*(double s) const { return {x * s, y * s}; }
    Vector2D operator/(double s) const { return {x / s, y / s}; }
    double norm() const { return std::sqrt(x * x + y * y); }
};

struct Bezier2D {
    Vector2D P0, P1, P2, P3;
    Vector2D position(double t) const;
    Vector2D d1(double t) const;
    Vector2D d2(double t) const;
    Vector2D d3() const;
    void getKinematics(double t, double T, Vector2D &pos, Vector2D &vel, Vector2D &acc) const;
};

struct TimeParam {
    std::vector<double> t;
    std::vector<double> dt_max; // allowed dt/ds at each sample
};

struct PathPoint {
    Vector2D pos, vel, acc;
    double time;
};

struct SegmentTime {
    double t_start;
    double t_end;
    TimeParam tp;
};

struct Path {
    std::vector<Bezier2D> segments;
    std::vector<SegmentTime> segment_times;

    Path(const std::vector<Vector2D>& waypoints,
         double v_max, double a_max, double j_max, int num_samples=200);

    PathPoint sample(double t) const;
    double getTotalTime() const {
        if (segment_times.empty()) return 0.0;
        return segment_times.back().t_end;
    }
    PathPoint getStartPoint() const {
        return sample(0.0);
    }
    PathPoint getEndPoint() const {
        return sample(getTotalTime());
    }
    PathPoint getPointAtIndex(size_t index) const {
        return sample(segment_times[index].t_start);
    }
};