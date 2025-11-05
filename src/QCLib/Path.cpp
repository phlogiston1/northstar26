#include "Path.h"
#include "Util.h"


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

