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

class State {
    private:
        Pose3D pose;
        Vector3D velocity;
        Vector3D angular_velocity;
        MotorVelocities motorVelocities;
        double time;

    public:
        State(Pose3D pose, Vector3D velocity, Vector3D angular_velocity, MotorVelocities motorVelocities, double time);
        Pose3D getPose();
        Vector3D getLinearVelocity();
        Vector3D getAngularVelocity();
        Vector3D getAngularVelocityLocal();
        MotorVelocities getMotorVelocities();
        void setMotorVelocities(MotorVelocities newVels);
        double getTime();
        void print();
        State predict(double timestep);
};

#endif