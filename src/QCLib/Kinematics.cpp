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
double arm_length = sqrt(2*rotor_pos*rotor_pos);

double velocityToThrust(double velocity) {
    return THRUST_COEFF * velocity * velocity;
}


Acceleration velocitiesToAccel (State currentState) {
    //Motor velocities
    double v_front = currentState.motor_velocities.getFront();
    double v_rear = currentState.motor_velocities.getRear();
    double v_left = currentState.motor_velocities.getLeft();
    double v_right = currentState.motor_velocities.getRight();

    //Thrusts from each rotor
    double thrust_front = velocityToThrust(v_front);
    double thrust_rear = velocityToThrust(v_rear);
    double thrust_left = velocityToThrust(v_left);
    double thrust_right = velocityToThrust(v_right);

    //calculate the torque on the quadcopter from differences in rotors speed
    double torque_pitch = (thrust_left - thrust_right) * arm_length;
    double torque_roll = (thrust_front - thrust_rear) * arm_length;

    //Each rotor also imparts rotation on the quadcopter
    double torque_yaw_front = (v_front) * ROTOR_DRAG_COEFF;
    double torque_yaw_rear = (v_rear) * ROTOR_DRAG_COEFF;
    double torque_yaw_left = (v_left) * ROTOR_DRAG_COEFF;
    double torque_yaw_right = (v_right ) * ROTOR_DRAG_COEFF;

    //pair up the rotors that are spinning in the same direction.
    //This assumes that the LEFT and RIGHT rotors are spinning CLOCKWISE.
    double yaw_torque_ccw = torque_yaw_left + torque_yaw_right;
    double yaw_torque_cw = torque_yaw_front + torque_yaw_rear;

    //overall yaw torque.
    double torque_yaw = yaw_torque_ccw - yaw_torque_cw;

    //drag due to current quadcopter rotation
    Vector3D drag_torque = Vector3D(
        currentState.angular_velocity.x * ANGULAR_DRAG_COEFF_XY,
        currentState.angular_velocity.y * ANGULAR_DRAG_COEFF_XY,
        currentState.angular_velocity.z * ANGULAR_DRAG_COEFF_Z
    );

    //divide my MOI to get acceleration
    //Note: Can't use Rotation3d because it can wrap.
    double accel_yaw = (torque_yaw - drag_torque.z) / QUADCOPTER_IZ;
    double accel_pitch = (torque_pitch - drag_torque.y) / QUADCOPTER_IXY;
    double accel_roll = (torque_roll - drag_torque.x) / QUADCOPTER_IXY;

    double thrust = thrust_front + thrust_rear + thrust_right + thrust_left;
    std::array thrustDirection = currentState.pose.rotation.thrustDirection();

    Vector3D force = Vector3D(
        thrust*thrustDirection[0],
        thrust*thrustDirection[1],
        thrust*thrustDirection[2]
    );

    auto vel = currentState.linear_velocity;
    auto velocity_squared  = Vector3D(
        vel.x * std::abs(vel.x),
        vel.y * std::abs(vel.y),
        vel.z * std::abs(vel.z)
    );
    Vector3D drag_force = velocity_squared.componentWiseMultiply(Vector3D(LINEAR_DRAG_COEFF_XY, LINEAR_DRAG_COEFF_XY, LINEAR_DRAG_COEFF_Z));

    Vector3D acceleration = (force - drag_force) / QUADCOPTER_MASS;
    acceleration.z -= G;

    return Acceleration(
        acceleration.x,
        acceleration.y,
        acceleration.z,
        accel_yaw,
        accel_pitch,
        accel_roll
    );
}