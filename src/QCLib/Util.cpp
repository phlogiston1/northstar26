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
static Vector3d rotateVectorByQuat(const Rotation3d& q, const Vector3d& v) {
    // q = (w, qv)
    Vector3d qv{ q.x, q.y, q.z };
    // t = 2 * cross(qv, v)
    Vector3d t = qv.cross(v) * 2.0;
    // v' = v + w * t + cross(qv, t)
    Vector3d res = v + (t * q.w) + qv.cross(t);
    return res;
}


// -----------------------------
// Rotation3d implementations
// -----------------------------
Rotation3d Rotation3d::fromRotationMatrix(const Vector3d& x_axis, const Vector3d& y_axis, const Vector3d& z_axis) {
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

    return Rotation3d(qw, qx, qy, qz);
}

Rotation3d Rotation3d::fromDegrees(double yaw, double pitch, double roll) {
    return Rotation3d(degreesToRadians(yaw), degreesToRadians(pitch), degreesToRadians(roll));
}

Vector3d Rotation3d::getZAxis() const {
    // Body Z axis in world coordinates = rotate (0,0,1) by quaternion
    Vector3d bodyZ{0.0, 0.0, 1.0};
    Vector3d worldZ = rotateVectorByQuat(*this, bodyZ);
    return worldZ.normalized();
}

Vector3d Rotation3d::getXAxis() const {
    Vector3d bodyX{1.0, 0.0, 0.0};
    Vector3d worldX = rotateVectorByQuat(*this, bodyX);
    return worldX.normalized();
}

Vector3d Rotation3d::getYAxis() const {
    Vector3d bodyY{0.0, 1.0, 0.0};
    Vector3d worldY = rotateVectorByQuat(*this, bodyY);
    return worldY.normalized();
}

std::array<double, 3> Rotation3d::thrustDirection() const {
    // Thrust acts along body -Z. Compute world direction of body -Z.
    Vector3d negBodyZ = rotateVectorByQuat(*this, Vector3d{0.0, 0.0, -1.0});
    double mag = negBodyZ.getMagnitude();
    if (mag == 0.0) return {0.0, 0.0, 0.0};
    return {negBodyZ.x / mag, negBodyZ.y / mag, negBodyZ.z / mag};
}

std::array<double, 3> Rotation3d::thrustVector(double thrustMagnitude) const {
    auto dir = thrustDirection();
    return { dir[0] * thrustMagnitude, dir[1] * thrustMagnitude, dir[2] * thrustMagnitude };
}


// -----------------------------
// Bezier2D implementations
// -----------------------------
Vector2D Bezier2D::position(double t) const {
    double u = 1.0 - t;
    double u2 = u * u;
    double u3 = u2 * u;
    double t2 = t * t;
    double t3 = t2 * t;
    return P0 * u3 + P1 * (3.0 * u2 * t) + P2 * (3.0 * u * t2) + P3 * t3;
}

Vector2D Bezier2D::d1(double t) const {
    double u = 1.0 - t;
    return (P1 - P0) * (3.0 * u * u)
         + (P2 - P1) * (6.0 * u * t)
         + (P3 - P2) * (3.0 * t * t);
}

Vector2D Bezier2D::d2(double t) const {
    // (P2 - 2*P1 + P0) * 6*(1-t) + (P3 - 2*P2 + P1) * 6*t
    Vector2D a = P2 - P1 * 2.0 + P0;
    Vector2D b = P3 - P2 * 2.0 + P1;
    return a * (6.0 * (1.0 - t)) + b * (6.0 * t);
}

Vector2D Bezier2D::d3() const {
    return (P3 - P2*3.0 + P1*3.0 - P0)*6.0;
}

void Bezier2D::getKinematics(double t, double T, Vector2D &pos, Vector2D &vel, Vector2D &acc) const {
    pos = position(t);
    vel = d1(t) * (1.0 / T);
    acc = d2(t) * (1.0 / (T * T));
}



/**
 * @brief Compute time parameterization for a 2D Bezier curve.
 * 
 * The time parameterization ensures that the curve can be traversed
 * within specified maximum velocity, acceleration, and jerk constraints.
 * It relates the curve parameter t to time increments dt.
 * 
 * @param curve the Bezier2D curve to parameterize
 * @param max_velocity the maximum allowed velocity
 * @param max_acceleration the maximum allowed acceleration
 * @param max_jerk the maximum allowed jerk
 * @param samples the number of samples to compute
 * @return TimeParam containing the time parameterization
 */
TimeParam computeTimeParameterization(
    const Bezier2D &curve,
    double max_velocity, double max_acceleration, double max_jerk,
    int samples = 200)
{
    TimeParam tp;
    tp.t.resize(samples);
    tp.dt_max.resize(samples);

    Vector2D d3_const = curve.d3();
    double d3_norm = d3_const.norm();

    for(int i = 0; i < samples; i++){
        double t = double(i)/(samples-1);
        Vector2D d1 = curve.d1(t);
        Vector2D d2 = curve.d2(t);

        double dt_vel = max_velocity / d1.norm();
        double dt_acc = std::sqrt(max_acceleration / d2.norm());
        double dt_jerk = std::cbrt(max_jerk / (d3_norm + 1e-6));

        tp.t[i] = t;
        tp.dt_max[i] = std::min(dt_vel, std::min(dt_acc, dt_jerk));
    }

    return tp;
}

std::vector<double> computeTimeSchedule(const TimeParam &tp) {
    std::vector<double> tau(tp.t.size());
    double time = 0.0;
    tau[0] = 0.0;

    for (size_t i = 1; i < tp.t.size(); i++) {
        double avg = (tp.dt_max[i] + tp.dt_max[i-1]) * 0.5;
        double dt = (tp.t[i] - tp.t[i-1]) / avg;
        time += dt;
        tau[i] = time;
    }

    return tau; // tau[i] = actual time at sample i
}

std::vector<PathPoint> generateFinalPath(
    const Bezier2D &curve,
    const TimeParam &tp,
    const std::vector<double> &tau)
{
    std::vector<PathPoint> path;
    path.reserve(tp.t.size());

    for(size_t i = 0; i < tp.t.size(); i++){
        double t = tp.t[i];
        double dt = tp.dt_max[i];
        Vector2D pos = curve.position(t);
        Vector2D vel = curve.d1(t) * dt;
        Vector2D acc = curve.d2(t) * dt * dt;
        path.push_back({pos, vel, acc, tau[i]});
    }
    return path;
}

std::vector<SegmentTime> computePathSegmentTimes(const std::vector<Bezier2D> &segments,
                                           double v_max, double a_max, double j_max, int num_samples=200)
{
    std::vector<SegmentTime> seg_times;
    double global_time = 0.0;

    for(const auto &seg : segments){
        TimeParam tp = computeTimeParameterization(seg, v_max, a_max, j_max, num_samples);
        // Integrate dt_max to get segment duration
        double seg_time = 0.0;
        for(size_t i = 1; i < tp.t.size(); i++){
            double avg = 0.5*(tp.dt_max[i] + tp.dt_max[i-1]);
            double dt = (tp.t[i] - tp.t[i-1]) / avg;
            seg_time += dt;
        }
        seg_times.push_back({global_time, global_time + seg_time, tp});
        global_time += seg_time;
    }
    return seg_times;
}

PathPoint evaluateChainedPath(const std::vector<Bezier2D> &segments,
                              const std::vector<SegmentTime> &seg_times,
                              double time)
{
    for(size_t i = 0; i < segments.size(); i++){
        const auto &seg = segments[i];
        const auto &st = seg_times[i];
        if(time <= st.t_end){
            double dt = st.tp.dt_max.front(); // approximation: sample could be interpolated
            double local_t = (time - st.t_start) / (st.t_end - st.t_start);
            Vector2D pos = seg.position(local_t);
            Vector2D vel = seg.d1(local_t) * dt;
            Vector2D acc = seg.d2(local_t) * dt * dt;
            return {pos, vel, acc, time};
        }
    }
    const auto &last = segments.back();
    return {last.P3, {0,0}, {0,0}, time};
}

Path::Path(const std::vector<Vector2D>& waypoints,
           double max_velocity, double max_acceleration, double max_jerk, int num_samples) {
    // Generate C1 chained segments (control points computed with Catmull-Rom tangents)
    auto tangent = [&](int i) -> Vector2D {
        if(i==0) return (waypoints[1]-waypoints[0]);
        if(i==waypoints.size()-1) return (waypoints.back()-waypoints[waypoints.size()-2]);
        return (waypoints[i+1]-waypoints[i-1])*0.5;
    };
    for(size_t i = 0; i < waypoints.size()-1; i++){
        Vector2D P0 = waypoints[i];
        Vector2D P3 = waypoints[i+1];
        Vector2D P1 = P0 + tangent(i)/3.0;
        Vector2D P2 = P3 - tangent(i+1)/3.0;
        segments.push_back({P0,P1,P2,P3});
    }

    segment_times = computePathSegmentTimes(segments, max_velocity, max_acceleration, max_jerk, num_samples);
}

PathPoint Path::sample(double time) const {
    for(size_t i = 0; i < segments.size(); i++){
        const auto &seg = segments[i];
        const auto &st = segment_times[i];
        if(time <= st.t_end){
            double dt = st.tp.dt_max.front(); // approximation: sample could be interpolated
            double local_t = (time - st.t_start) / (st.t_end - st.t_start);
            Vector2D pos = seg.position(local_t);
            Vector2D vel = seg.d1(local_t) * dt;
            Vector2D acc = seg.d2(local_t) * dt * dt;
            return {pos, vel, acc, time};
        }
    }
    const auto &last = segments.back();
    return {last.P3, {0,0}, {0,0}, time};
}

