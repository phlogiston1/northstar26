#include "Physics.h"
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

State::State (Pose3D pose, Vector3D velocity, Vector3D angular_velocity, MotorVelocities motorVelocities, double time): pose(pose), linear_velocity(velocity), angular_velocity(angular_velocity), motor_velocities(motorVelocities), time(time){

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

State State::fullKinematicsStep(double timestep) {
    Acceleration accel = velocitiesToAccel(*this);

    //calculate velocity and angular velocity
    double vel_x = (linear_velocity.x + (accel.getX() * timestep));
    double vel_y = (linear_velocity.y + (accel.getY() * timestep));
    double vel_z = (linear_velocity.z + (accel.getZ() * timestep));
    double ang_z = angular_velocity.z + (accel.getYaw() * timestep);
    double ang_y = angular_velocity.y + (accel.getPitch() * timestep);
    double ang_x = angular_velocity.x + (accel.getRoll() * timestep);

    //calculate position and angular position.
    //integrates under the velocity curve for more accuracy
    double pos_x = pose.getX() + (linear_velocity.x * timestep) + (0.5 * timestep * timestep * accel.getX());
    double pos_y = pose.getY() + (linear_velocity.y * timestep) + (0.5 * timestep * timestep * accel.getY());
    double pos_z = pose.getZ() + (linear_velocity.z * timestep) + (0.5 * timestep * timestep * accel.getZ());
    double ang_pos_z = pose.rotation.getYaw() + (angular_velocity.z * timestep) + (0.5 * timestep * timestep * accel.getYaw());
    double ang_pos_y = pose.rotation.getPitch() + (angular_velocity.y * timestep) + (0.5 * timestep * timestep * accel.getPitch());
    double ang_pos_x = pose.rotation.getRoll() + (angular_velocity.x * timestep) + (0.5 * timestep * timestep * accel.getRoll());

    //Don't go underground
    if(ENABLE_FLOOR && pos_z < 0) {
        pos_z = 0;
        if(vel_z < 0) vel_z = 0;
    }

    return State(
        Pose3D(Vector3D(pos_x,pos_y,pos_z), Quaternion(ang_pos_z,ang_pos_y,ang_pos_x)),
        Vector3D(vel_x,vel_y,vel_z),
        Vector3D(ang_x, ang_y, ang_z),
        motor_velocities,
        time+timestep
    );
}

State State::translationKinematicsStep(double timestep) {
    Acceleration accel = velocitiesToAccel(*this);
    //calculate velocity and angular velocity
    double vel_x = (linear_velocity.x + (accel.getX() * timestep));
    double vel_y = (linear_velocity.y + (accel.getY() * timestep));
    double vel_z = (linear_velocity.z + (accel.getZ() * timestep));

    //calculate position and angular position.
    //integrates under the velocity curve for more accuracy
    double pos_x = pose.getX() + (linear_velocity.x * timestep) + (0.5 * timestep * timestep * accel.getX());
    double pos_y = pose.getY() + (linear_velocity.y * timestep) + (0.5 * timestep * timestep * accel.getY());
    double pos_z = pose.getZ() + (linear_velocity.z * timestep) + (0.5 * timestep * timestep * accel.getZ());

    //Don't go underground
    if(ENABLE_FLOOR && pos_z < 0) {
        pos_z = 0;
        if(vel_z < 0) vel_z = 0;
    }

    return State(
        Pose3D(Vector3D(pos_x,pos_y,pos_z), pose.rotation),
        Vector3D(vel_x,vel_y,vel_z),
        angular_velocity,
        motor_velocities,
        time+timestep
    );
}

void State::print() {
    std::cout << "\n\nQuadcopter State at t=" << time << "\n";
    std::cout << "POSE - "; pose.print();
    std::cout << "VELOCITY - "; linear_velocity.print();
    std::cout << "ANG VELOCITY - "; angular_velocity.print();
    std::cout << "MOTOR VELS - left: " << motor_velocities.getLeft() 
                          << " front: " << motor_velocities.getFront()
                          << " right: " << motor_velocities.getRight()
                          << " rear: " << motor_velocities.getRear();
    std::cout << "\n\n";
}