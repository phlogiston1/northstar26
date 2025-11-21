#include <array>
#include "Util.h"

#ifndef QUADCOPTER_H
#define QUADCOPTER_H
//ALL POSITIVE VALUES, IGNORING DIRECTION
class MotorVelocities {
    private:
        double left;
        double front;
        double right;
        double rear;

    public:
        MotorVelocities(double left, double front, double right, double rear);

        double getLeft() const;
        double getFront() const;
        double getRight() const;
        double getRear() const;

        MotorVelocities limit(double maxVelocity) const;
};



class Acceleration {
    private:
        double x, y, z, yaw, pitch, roll;

    public:
        Acceleration(double x, double y, double z, double yaw, double pitch, double roll);

        double getX();
        double getY();
        double getZ();
        double getYaw();
        double getPitch();
        double getRoll();
};

struct State {
    Pose3D pose;
    Vector3D linear_velocity;
    Vector3D angular_velocity;
    MotorVelocities motor_velocities;
    double time;

    State(Pose3D pose, Vector3D velocity, Vector3D angular_velocity, MotorVelocities motorVelocities, double time);
    Vector3D getAngularVelocityLocal();
    void print();
    State fullKinematicsStep(double timestep);
    State translationKinematicsStep(double timestep);
};
#endif