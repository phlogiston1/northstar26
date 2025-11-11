#include <array>
#include "Util.h"

#ifndef QUADCOPTER_H
#define QUADCOPTER_H
//ALL POSITIVE VALUES, IGNORING DIRECTION
class MotorVelocities {
    private:
        double frontLeft;
        double frontRight;
        double rearLeft;
        double rearRight;

    public:
        MotorVelocities(double frontLeft, double frontRight, double rearLeft, double rearRight);

        double getFrontLeft() const;
        double getFrontRight() const;
        double getRearLeft() const;
        double getRearRight() const;

        MotorVelocities limit(double maxVelocity) const;
};



class QCAcceleration {
    private:
        Rotation3d angular_accel;
        double accel_x;
        double accel_y;
        double accel_z;

    public:
        QCAcceleration(Rotation3d angular_accel, double accel_x, double accel_y, double accel_z);

        Rotation3d getAngular();
        double getX();
        double getY();
        double getZ();
};

class QCState {
    private:
        Pose3d pose;
        Pose3d velocity;
        MotorVelocities motorVelocities;
        double time;

    public:
        QCState(Pose3d pose, Pose3d velocity, MotorVelocities motorVelocities, double time);
        Pose3d getPose();
        Pose3d getVelocity();
        MotorVelocities getMotorVelocities();
        void setMotorVelocities(MotorVelocities newVels);
        double getTime();
        void print();
        QCState predict(double timestep);
};

#endif