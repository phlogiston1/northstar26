#pragma once
#include "Physics.h"
#include "Path.h"
#include "InverseKinematics.h"
#include <chrono>


struct QCRequest{
    Pose3D position, velocity;
    QCRequest(Pose3D position, Pose3D velocity): position(position), velocity(velocity) {

    }
};

class VelocityController{
    public:
        VelocityController();
        void setInitialPose(State current);
        QCRequest getTarget(State current, Vector3D target_velocity);
    private:
        Vector3D velocity = Vector3D(0,0,0);
        Vector3D pose = Vector3D(0,0,0);
};

class PathController{
    public:
        PathController();
        void beginPath(const Path& newPath, double cruiseHeight);
        QCRequest getTarget(State current);
        QCRequest getTarget(State current, double yaw);
        Vector3D getTargetAcceleration(State& currentState, Pose3D currentPosition);
        bool complete();
    private:
        Path path;
        double cruiseHeight;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
};

//uses trapeoidal motion profile to smoothly take off to a target height
class HeightController{
    public:
        HeightController();
        void setTargetHeight(double height, double currentHeight);
        QCRequest getTarget(State& currentState);
        bool complete();
    private:
        double start_height;
        double delta_height;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
        double accel_time; //time spent accelerating
        double cruise_time; //time spent at constant velocity
        double deceleration_time; //time spent decelerating
        bool inverted = false;
};