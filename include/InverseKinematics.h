#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "Quadcopter.h"

/**
 * @brief Implements inverse kinematics for the quadcopter
 *
 * Step 1: Given a desired acceleration vector and yaw rate, calculate the ideal orientation and thrust for the quadcopter, 
 * ignoring frame and motor acceleration limits.
 * Step 2: Given the ideal orientation and thrust, calculate the motor velocities that would best achieve this
 * We will have two different kinematics modes, one will adjust the thrust to best match the *direction* of the acceleration vector,
 * and the other will adjust the thrust to best match the *magnitude* of the acceleration vector.
 * I'm not sure which will work better, so I'll implement both and see.
 */

/**
 * @brief Represents the result of an inverse kinematics calculation
 */
 struct InverseKinematicResult {
    MotorVelocities motorVelocities;
    Acceleration achievedAccel;
    double errorMagnitude;
};


/**
 * @brief Represents the target state for the quadcopter based on desired acceleration and yaw rate
 */
struct TargetQCState {
    Quaternion targetAngle;
    double targetThrust;
    double targetYawRate;
};

Quaternion getTargetAngle(State currentState, Vector3D targetAccel);


#endif