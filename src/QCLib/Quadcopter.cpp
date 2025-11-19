#include "Quadcopter.h"
#include "Kinematics.h"
#include "Util.h"
#include "Configuration.h"
#include <array>
#include <iostream>
#include <cmath>


MotorVelocities::MotorVelocities(double left, double front, double right, double rear) : left(left), front(front), right(right), rear(rear) {

}

double MotorVelocities::getLeft() const {
    return left;
}

double MotorVelocities::getFront() const {
    return front;
}

double MotorVelocities::getRear() const {
    return rear;
}

double MotorVelocities::getRight() const {
    return right;
}

MotorVelocities MotorVelocities::limit(double maxVelocity) const {
    return MotorVelocities(
        std::min(left, maxVelocity),
        std::min(front, maxVelocity),
        std::min(right, maxVelocity),
        std::min(rear, maxVelocity)
    );
}


Acceleration::Acceleration(double x, double y, double z, double yaw, double pitch, double roll):
    x(x),
    y(y),
    z(z),
    yaw(yaw),
    pitch(pitch),
    roll(roll){

}


double Acceleration::getX() {
    return x;
}

double Acceleration::getY() {
    return y;
}

double Acceleration::getZ() {
    return z;
}

double Acceleration::getYaw() {
    return yaw;
}

double Acceleration::getPitch() {
    return pitch;
}

double Acceleration::getRoll() {
    return roll;
}

State::State (Pose3D pose, Vector3D velocity, Vector3D angular_velocity, MotorVelocities motorVelocities, double time): pose(pose), velocity(velocity), angular_velocity(angular_velocity), motorVelocities(motorVelocities), time(time){

}

Pose3D State::getPose() {
    return pose;
}

Vector3D State::getLinearVelocity() {
    return velocity;
}

Vector3D State::getAngularVelocity() {
    return angular_velocity;
}

Vector3D State::getAngularVelocityLocal() {
    double roll  = pose.rotation.getRoll();
    double pitch = pose.rotation.getPitch();
    double yaw   = pose.rotation.getYaw();

    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);

    // Rotation matrix R = Rz * Ry * Rx
    double R[3][3];
    R[0][0] = cy*cp;
    R[0][1] = cy*sp*sr - sy*cr;
    R[0][2] = cy*sp*cr + sy*sr;

    R[1][0] = sy*cp;
    R[1][1] = sy*sp*sr + cy*cr;
    R[1][2] = sy*sp*cr - cy*sr;

    R[2][0] = -sp;
    R[2][1] = cp*sr;
    R[2][2] = cp*cr;

    // Local angular velocity = R^T * w_world
    Vector3D w_local;
    w_local.x = R[0][0]*angular_velocity.x + R[1][0]*angular_velocity.y + R[2][0]*angular_velocity.z;
    w_local.y = R[0][1]*angular_velocity.x + R[1][1]*angular_velocity.y + R[2][1]*angular_velocity.z;
    w_local.z = R[0][2]*angular_velocity.x + R[1][2]*angular_velocity.y + R[2][2]*angular_velocity.z;

    return w_local;
}

MotorVelocities State::getMotorVelocities() {
    return motorVelocities;
}

void State::setMotorVelocities(MotorVelocities newVels) {
    motorVelocities = newVels;
}

double State::getTime(){
    return time;
}

State State::predict(double timestep) {
    Acceleration accel = velocitiesToAccel(*this);
    std::cout << "PREDITED ACCEL - x: " << accel.getX() << " y: " << accel.getY() << " z: " << accel.getZ() << std::endl;

    // double drag_x = -velocity.getX() * LINEAR_DRAG_COEFF_XY;
    // double drag_y = -velocity.getY() * LINEAR_DRAG_COEFF_XY;
    // double drag_z = -velocity.getZ() * LINEAR_DRAG_COEFF_Z;

    // std::cout << "Drag: " << drag_z << " Z Velocity: " << velocity.getZ() << "\n";

    // double ang_drag_x = -velocity.rotation.getRoll() * ANGULAR_DRAG_COEFF_XY;
    // double ang_drag_y = -velocity.rotation.getPitch() * ANGULAR_DRAG_COEFF_XY;
    // double ang_drag_z = -velocity.rotation.getYaw() * ANGULAR_DRAG_COEFF_Z;

    double newVX = (velocity.x + (accel.getX() * timestep));
    double newVY = (velocity.y + (accel.getY() * timestep));
    double newVZ = (velocity.z + (accel.getZ() * timestep));

    double newAZ = angular_velocity.z + ((accel.getYaw()/* + ang_drag_z*/) * timestep);
    double newAY = angular_velocity.y + ((accel.getPitch()/* + ang_drag_y*/) * timestep);
    double newAX = angular_velocity.x + ((accel.getRoll()/* + ang_drag_x*/) * timestep);

    double newPX = pose.getX() + (velocity.x * timestep) + (0.5 * timestep * timestep * accel.getX());
    double newPY = pose.getY() + (velocity.y * timestep) + (0.5 * timestep * timestep * accel.getY());
    double newPZ = pose.getZ() + (velocity.z * timestep) + (0.5 * timestep * timestep * accel.getZ());
    double newPAZ = pose.rotation.getYaw() + (newAZ * timestep);
    double newPAY = pose.rotation.getPitch() + (newAY * timestep);
    double newPAX = pose.rotation.getRoll() + (newAX * timestep);

    if(ENABLE_FLOOR && newPZ < 0) {
        newPZ = 0; //don't go underground
        if(newVZ < 0) newVZ = 0;
    }

    return State(
        Pose3D(Vector3D(newPX,newPY,newPZ), Quaternion(newPAZ,newPAY,newPAX)),
        Vector3D(newVX,newVY,newVZ),
        Vector3D(newAX, newAY, newAZ),
        motorVelocities,
        time+timestep
    );
}

void State::print() {
    std::cout << "\n\nQuadcopter State at t=" << time << "\n";
    std::cout << "POSE - "; pose.print();
    std::cout << "VELOCITY - "; velocity.print();
    std::cout << "ANG VELOCITY - "; angular_velocity.print();
    std::cout << "MOTOR VELS - left: " << motorVelocities.getLeft() 
                          << " front: " << motorVelocities.getFront()
                          << " right: " << motorVelocities.getRight()
                          << " rear: " << motorVelocities.getRear();
    std::cout << "\n\n";
}