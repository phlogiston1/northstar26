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


extern "C" {


    Quadcopter* Quadcopter_new() {
        State initialState = State(
            Pose3D(Vector3D(0,0,0), Quaternion(0,0,0)),
            Vector3D(0,0,0),
            Vector3D(0,0,0),
            MotorVelocities(0,0,0,0),
            0
        );
        return new Quadcopter(initialState, 1,1,0.3);
    };

    void Quadcopter_setHeight(Quadcopter* obj, double height) {obj ->setHeight(height);}

    void Quadcopter_update_simulation(Quadcopter* obj) {obj -> update_simulation();}
    void Quadcopter_printState(Quadcopter* obj) {obj -> getState().print();}

    double Quadcopter_frontMotorVel(Quadcopter* obj) {return obj->getMotorVels().getFront();}
    double Quadcopter_leftMotorVel(Quadcopter* obj) {return obj->getMotorVels().getLeft();}
    double Quadcopter_rearMotorVel(Quadcopter* obj) {return obj->getMotorVels().getRear();}
    double Quadcopter_rightMotorVel(Quadcopter* obj) {return obj->getMotorVels().getRight();}

    double Quadcopter_getTime(Quadcopter* obj) {return obj->getTime();}
};