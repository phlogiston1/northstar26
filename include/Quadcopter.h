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
        Vector3D manual_velocity = Vector3D();
        QCRequest req = QCRequest(Pose3D(), Pose3D());
        bool manual = false;
        LandingStatus landing_status = LANDED;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
        std::vector<Vector2D> path_waypoints = {};

        VelocityController velocity_controller;
        PathController path_controller;
        HeightController height_controller;


    public:
        Quadcopter(State initial);
        State getState();
        void addVisionMeasurement(Vector3D translation, double timestamp);
        void addIMUMeasurement(Quaternion angular_pos, Vector3D angular_vel);
        void setHeight(double height);
        void addWaypoint(Vector2D waypoint);
        void beginPath();
        void beginPath(Path path);
        void beginManualControl();
        void setVelocity(Vector3D velocity);
        void land();
        bool busy();
        bool adjustingHeight();
        //these return pointers to allow python binding
        QCRequest* getRequest();
        Vector3D* getTranslation();
        Vector3D* getVelocity();
        void update_simulation();
        double getTime();
        MotorVelocities getMotorVels();
};


extern "C" {
    Vector3D* Vector3D_new(double x, double y, double z){return new Vector3D(x,y,z);}
    double Vector3D_x(Vector3D* obj) {return obj->x;}
    double Vector3D_y(Vector3D* obj) {return obj->y;}
    double Vector3D_z(Vector3D* obj) {return obj->z;}

    Quaternion* Quaternion_new(double yaw, double pitch, double roll){return new Quaternion(yaw,pitch,roll);}
    double Quaternion_w(Quaternion* obj) {return obj->w;}
    double Quaternion_x(Quaternion* obj) {return obj->x;}
    double Quaternion_y(Quaternion* obj) {return obj->y;}
    double Quaternion_z(Quaternion* obj) {return obj->z;}
    double Quaternion_getYaw(Quaternion* obj) {return obj->getYaw();}
    double Quaternion_getPitch(Quaternion* obj) {return obj->getPitch();}
    double Quaternion_getRoll(Quaternion* obj) {return obj->getRoll();}

    Pose3D* Pose3D_new(Vector3D* vec, Quaternion* quat) {return new Pose3D(*vec, *quat);}
    Vector3D* Pose3D_translation(Pose3D* obj){return &obj->translation;}
    Quaternion* Pose3D_rotation(Pose3D* obj){return &obj->rotation;}

    QCRequest* QCRequest_new(Pose3D* position, Pose3D* velocity) {return new QCRequest(*position, *velocity);}
    Pose3D* QCRequest_position(QCRequest* obj){return &obj->position;}
    Pose3D* QCRequest_velocity(QCRequest* obj){return &obj->velocity;}

    Quadcopter* Quadcopter_new() {
        State initialState = State(
            Pose3D(Vector3D(0,0,0), Quaternion(0,0,0)),
            Vector3D(0,0,0),
            Vector3D(0,0,0),
            MotorVelocities(0,0,0,0),
            0
        );
        return new Quadcopter(initialState);
    };

    void Quadcopter_setHeight(Quadcopter* obj, double height) {obj ->setHeight(height);}

    void Quadcopter_update_simulation(Quadcopter* obj) {obj -> update_simulation();}
    void Quadcopter_printState(Quadcopter* obj) {obj -> getState().print();}

    double Quadcopter_frontMotorVel(Quadcopter* obj) {return obj->getMotorVels().getFront();}
    double Quadcopter_leftMotorVel(Quadcopter* obj) {return obj->getMotorVels().getLeft();}
    double Quadcopter_rearMotorVel(Quadcopter* obj) {return obj->getMotorVels().getRear();}
    double Quadcopter_rightMotorVel(Quadcopter* obj) {return obj->getMotorVels().getRight();}

    Vector3D* Quadcopter_getTranslation(Quadcopter* obj) {return obj->getTranslation();};
    Vector3D* Quadcopter_getVelocity(Quadcopter* obj) {return obj->getVelocity();};
    double Quadcopter_getTime(Quadcopter* obj) {return obj->getTime();}
    QCRequest* Quadcopter_getRequest(Quadcopter* obj) {return obj->getRequest();}

    void Quadcopter_addWaypoint(Quadcopter* obj, double x, double y) {obj->addWaypoint(Vector2D{x,y});}
    void Quadcopter_beginPath(Quadcopter* obj) {obj->beginPath();}

    void Quadcopter_beginManualControl(Quadcopter* obj) {obj->beginManualControl();}
    void Quadcopter_setVelocity(Quadcopter* obj, Vector3D velocity) {obj->setVelocity(velocity);}

    bool Quadcopter_busy(Quadcopter* obj) {return obj->busy();}
};