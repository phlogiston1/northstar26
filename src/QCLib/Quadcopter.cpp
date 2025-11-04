#include "Quadcopter.h"
#include "Kinematics.h"
#include "Util.h"
#include "Configuration.h"
#include <array>
#include <iostream>
#include <cmath>


MotorVelocities::MotorVelocities(double fl, double fr, double rl, double rr) : frontLeft(fl), frontRight(fr), rearLeft(rl), rearRight(rr) {

}

double MotorVelocities::getFrontLeft() const {
    return frontLeft;
}

double MotorVelocities::getFrontRight() const {
    return frontRight;
}

double MotorVelocities::getRearRight() const {
    return rearRight;
}

double MotorVelocities::getRearLeft() const {
    return rearLeft;
}

MotorVelocities MotorVelocities::limit(double maxVelocity) const {
    return MotorVelocities(
        std::min(frontLeft, maxVelocity),
        std::min(frontRight, maxVelocity),
        std::min(rearLeft, maxVelocity),
        std::min(rearRight, maxVelocity)
    );
}


QCAcceleration::QCAcceleration(Rotation3d angular_accel, double accel_x, double accel_y, double accel_z):
    angular_accel(angular_accel),
    accel_x(accel_x),
    accel_y(accel_y),
    accel_z(accel_z){

}

Rotation3d QCAcceleration::getAngular() {
    return angular_accel;
}

double QCAcceleration::getX() {
    return accel_x;
}

double QCAcceleration::getY() {
    return accel_y;
}

double QCAcceleration::getZ() {
    return accel_z;
}

QCState::QCState(Pose3d pose, Pose3d velocity, MotorVelocities motorVelocities, double time): pose(pose), velocity(velocity), motorVelocities(motorVelocities), time(time){

}

Pose3d QCState::getPose() {
    return pose;
}

Pose3d QCState::getVelocity() {
    return velocity;
}

MotorVelocities QCState::getMotorVelocities() {
    return motorVelocities;
}

void QCState::setMotorVelocities(MotorVelocities newVels) {
    motorVelocities = newVels;
}

double QCState::getTime(){
    return time;
}

QCState QCState::predict(double timestep) {
    QCAcceleration accel = velocitiesToAccel(motorVelocities, pose.rotation);
    double drag_x = -velocity.getX() * LINEAR_DRAG_COEFF_XY;
    double drag_y = -velocity.getY() * LINEAR_DRAG_COEFF_XY;
    double drag_z = -velocity.getZ() * LINEAR_DRAG_COEFF_Z;

    // std::cout << "Drag: " << drag_z << " Z Velocity: " << velocity.getZ() << "\n";

    double ang_drag_x = -velocity.rotation.getRoll() * ANGULAR_DRAG_COEFF_XY;
    double ang_drag_y = -velocity.rotation.getPitch() * ANGULAR_DRAG_COEFF_XY;
    double ang_drag_z = -velocity.rotation.getYaw() * ANGULAR_DRAG_COEFF_Z;

    double newVX = (velocity.getX() + ((accel.getX() + drag_x) * timestep));
    double newVY = (velocity.getY() + ((accel.getY() + drag_y) * timestep));
    double newVZ = (velocity.getZ() + ((accel.getZ() + drag_z) * timestep));

    double newAZ = velocity.rotation.getYaw() + ((accel.getAngular().getYaw() + ang_drag_z) * timestep);
    double newAY = velocity.rotation.getPitch() + ((accel.getAngular().getPitch() + ang_drag_y) * timestep);
    double newAX = velocity.rotation.getRoll() + ((accel.getAngular().getRoll() + ang_drag_x) * timestep);

    double newPX = pose.getX() + (newVX * timestep);
    double newPY = pose.getY() + (newVY * timestep);
    double newPZ = pose.getZ() + (newVZ * timestep);
    double newPAZ = pose.rotation.getYaw() + (newAZ * timestep);
    double newPAY = pose.rotation.getPitch() + (newAY * timestep);
    double newPAX = pose.rotation.getRoll() + (newAX * timestep);

    if(newPZ > 0) newPZ = 0; //don't go underground

    return QCState(
        Pose3d(Vector3d(newPX,newPY,newPZ), Rotation3d(newPAZ,newPAY,newPAX)),
        Pose3d(Vector3d(newVX,newVY,newVZ), Rotation3d(newAZ, newAY, newAX)),
        motorVelocities,
        time+timestep
    );
}