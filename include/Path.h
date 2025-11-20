#pragma once
#include <array>
#include <cmath>
#include <iostream>
#include <vector>
#include "Util.h"

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

    static Path origin();
};