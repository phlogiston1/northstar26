/**
 * @file Kinematics.cpp
 * @author Sean Benham
 * @brief Converts known motor thrusts to overall quadcopter acceleration
 * @version 0.1
 * @date 2025-10-12
 *
 * @copyright Sean Benham (c) 2025
 *
 */

#include "Kinematics.h"
#include "Configuration.h"
#include "Util.h"
#include <cmath>
#include <array>
#include <iostream>

double rotor_pos = QUADCOPTER_ROTOR_DISTANCE/2;

double velocityToThrust(double velocity) {
    return THRUST_COEFF * velocity * velocity;
}

QCAcceleration velocitiesToAccel (QCState currentState) {
    double thrust_fl = velocityToThrust(currentState.getMotorVelocities().getFrontLeft());
    double thrust_rl = velocityToThrust(currentState.getMotorVelocities().getRearLeft());
    double thrust_fr = velocityToThrust(currentState.getMotorVelocities().getFrontRight());
    double thrust_rr = velocityToThrust(currentState.getMotorVelocities().getRearRight());

    //note: torque_x means torque about the x axis, etc.
    //since the rotors are opposing each other's torques, we can simply consider the difference between opposite rotors.
    double torque_x = ((thrust_rl - thrust_fr) - (thrust_rr - thrust_fl)) * rotor_pos;
    double torque_y = ((thrust_rl - thrust_fr) + (thrust_rr - thrust_fl)) * rotor_pos;

    //calculate torque about the z axis due to drag from rotors:
    double drag_torque_fl = (currentState.getMotorVelocities().getFrontLeft() * currentState.getMotorVelocities().getFrontLeft()) * ROTOR_DRAG_COEFF;
    double drag_torque_rl = (currentState.getMotorVelocities().getRearLeft() * currentState.getMotorVelocities().getRearLeft()) * ROTOR_DRAG_COEFF;
    double drag_torque_fr = (currentState.getMotorVelocities().getFrontRight() * currentState.getMotorVelocities().getFrontRight()) * ROTOR_DRAG_COEFF;
    double drag_torque_rr = (currentState.getMotorVelocities().getRearRight() * currentState.getMotorVelocities().getRearRight()) * ROTOR_DRAG_COEFF;

    double drag_torque_a = drag_torque_fl + drag_torque_rr;
    double drag_torque_b = drag_torque_fr + drag_torque_rl;

    double torque_z;
    if(FRONT_LEFT_SPINS_CCW) torque_z = drag_torque_b - drag_torque_a;
    else torque_z = drag_torque_a - drag_torque_b;

    //calculate angular acceleration from all of the torques
    Rotation3d angularAccel = Rotation3d(torque_z / QUADCOPTER_MOI, torque_y / QUADCOPTER_MOI, torque_x / QUADCOPTER_MOI);

    //finally, calculate overall thrust and then rotate it to align with the measured quadcopter angle. Using spherical coordinates.
    double thrust = thrust_fl + thrust_rl + thrust_fr + thrust_rr;
    std::array thrustDirection = currentState.getPose().rotation.thrustDirection();

    Vector3D force = Vector3D(
        thrust*thrustDirection[0],
        thrust*thrustDirection[1],
        thrust*thrustDirection[2]
    );

    auto vel = currentState.getVelocity().translation;
    auto velocity_squared  = Vector3D(
        vel.x * std::abs(vel.x),
        vel.y * std::abs(vel.y),
        vel.z * std::abs(vel.z)
    );
    Vector3D dragForce = velocity_squared.componentWiseMultiply(Vector3D(LINEAR_DRAG_COEFF_XY, LINEAR_DRAG_COEFF_XY, LINEAR_DRAG_COEFF_Z));

    Vector3D acceleration = (force - dragForce) / QUADCOPTER_MASS;
    acceleration.z += G;

    return QCAcceleration(
        angularAccel,
        acceleration.x,
        acceleration.y,
        acceleration.z
    );

}