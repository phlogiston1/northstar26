#pragma once

#include "Physics.h"
#include "MotionController.h"
#include "Path.h"
#include <vector>
#include <functional>


struct PoseObservation {
    Pose3D pose = Pose3D();
    double timestamp = 0;
};

enum LandingStatus{
    FLYING,
    DECENT,
    TOUCHDOWN,
    LANDED
};



class Quadcopter {
    private:
        State state;
        Vector3D global_observation = Vector3D();
        std::vector<PoseObservation> predictions = {};
        std::function<Vector3D()> velocity_supplier;
        bool manual = false;
        LandingStatus landing_status = LANDED;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();

        VelocityController velocity_controller;
        PathController path_controller;
        HeightController height_controller;


    public:
        Quadcopter(State initial, double max_velocity, double max_acceleration, double max_jerk);
        State getState();
        void addVisionMeasurement(Vector3D translation, double timestamp);
        void addIMUMeasurement(Quaternion angular_pos, Vector3D angular_vel);
        void setHeight(double height);
        void beginPath(Path path);
        void beginManualControl(std::function<Vector3D()> velocity);
        void land();
        bool busy();
        bool adjustingHeight();
        void update_simulation();
        double getTime();
        MotorVelocities getMotorVels();
};