#include "MotionController.h"
#include "Configuration.h"
#include "Util.h"
#include <cmath>

/**
 * @file MotionController.cpp
 * @author Sean Benham
 * @brief Implements various motion controllers for the quadcopter
 * @version 0.1
 * @date 2025-10-12
 * The motion controllers determine the acceleration of the quadcopter based on the desired state of the copter.
 * There are three types. VelocityController is for manual input. It limits acceleration when attaining a target velocity, probably from a joystick.
 * PathController is for following a predefined path. It uses a simple P controller to follow the path while maintaining a cruise height.
 * 
 */

 //ManualController simply sets the motor velocities to a fixed value
VelocityController::VelocityController(){
}

Vector3d VelocityController::getTargetAcceleration(QCState& currentState, const Vector3d& targetVelocity) {
    Vector3d targetAcceleration = targetVelocity - currentState.getVelocity().translation;
    //limit acceleration based on max acceleration constants
    //it would be theoretically better to use polar coordinates here, but this is less computationally intensive
    if (targetAcceleration.x > MAX_ACCELERATION_XY) targetAcceleration.x = MAX_ACCELERATION_XY;
    if (targetAcceleration.x < -MAX_ACCELERATION_XY) targetAcceleration.x = -MAX_ACCELERATION_XY;
    if (targetAcceleration.y > MAX_ACCELERATION_XY) targetAcceleration.y = MAX_ACCELERATION_XY;
    if (targetAcceleration.y < -MAX_ACCELERATION_XY) targetAcceleration.y = -MAX_ACCELERATION_XY;
    if (targetAcceleration.z > MAX_ACCELERATION_Z) targetAcceleration.z = MAX_ACCELERATION_Z;
    if (targetAcceleration.z < -MAX_ACCELERATION_Z) targetAcceleration.z = -MAX_ACCELERATION_Z;
    //smooth acceleration changes based on max jerk constants. Z axis doesn't need jerk constraint.
    Vector3d jerk = targetAcceleration - lastTargetAcceleration;
    if (jerk.x > MAX_JERK_XY) jerk.x = MAX_JERK_XY;
    if (jerk.x < -MAX_JERK_XY) jerk.x = -MAX_JERK_XY;
    if (jerk.y > MAX_JERK_XY) jerk.y = MAX_JERK_XY;
    if (jerk.y < -MAX_JERK_XY) jerk.y = -MAX_JERK_XY;
    targetAcceleration = lastTargetAcceleration + jerk;
    lastTargetAcceleration = targetAcceleration;
    return targetAcceleration;
}

PathController::PathController(Vector2D position_kp, Vector2D velocity_kp, double cruiseHeight_kP)
    : position_kp(position_kp), velocity_kp(velocity_kp), cruiseHeight_kP(cruiseHeight_kP), path(Path({Vector2D{0,0}}, MAX_VELOCITY_XY, MAX_ACCELERATION_XY, MAX_JERK_XY, 2)) {
}

void PathController::beginPath(const Path& newPath, double cruiseHeight) {
    path = newPath;
    this->cruiseHeight = cruiseHeight;
    startTime = std::chrono::high_resolution_clock::now();
}

Vector3d PathController::getTargetAcceleration(QCState& currentState, Pose3d currentPosition) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    PathPoint targetPoint = path.sample(elapsed);
    Vector2D positionError = targetPoint.pos - currentPosition.translation.toVector2D();
    Vector2D velocityError = targetPoint.vel - currentState.getVelocity().translation.toVector2D();

    //modify velocity error based on position error and position kp
    velocityError = velocityError + Vector2D{
        positionError.x * position_kp.x,
        positionError.y * position_kp.y
    };

    Vector3d targetAcceleration = Vector3d{
        velocityError.x * velocity_kp.x + targetPoint.acc.x,
        velocityError.y * velocity_kp.y + targetPoint.acc.y,
        (cruiseHeight - currentPosition.getZ()) * cruiseHeight_kP
    };

    //limit acceleration based on max acceleration constants
    if (targetAcceleration.x > MAX_ACCELERATION_XY) targetAcceleration.x = MAX_ACCELERATION_XY;
    if (targetAcceleration.x < -MAX_ACCELERATION_XY) targetAcceleration.x = -MAX_ACCELERATION_XY;
    if (targetAcceleration.y > MAX_ACCELERATION_XY) targetAcceleration.y = MAX_ACCELERATION_XY;
    if (targetAcceleration.y < -MAX_ACCELERATION_XY) targetAcceleration.y = -MAX_ACCELERATION_XY;
    if (targetAcceleration.z > MAX_ACCELERATION_Z) targetAcceleration.z = MAX_ACCELERATION_Z;
    if (targetAcceleration.z < -MAX_ACCELERATION_Z) targetAcceleration.z = -MAX_ACCELERATION_Z;
    return targetAcceleration;
}

TakeoffController::TakeoffController(double kP, double velocity, double acceleration) : kP(kP), maxVelocity(velocity), maxAcceleration(acceleration) {
    setTargetHeight(0.0, 0.0);
}

void TakeoffController::setTargetHeight(double height, double currentHeight) {
    targetHeight = height;
    startTime = std::chrono::high_resolution_clock::now();
    //calculate motion profile times based on max acceleration and velocity
    double distance = targetHeight - currentHeight;
    acceleration_time = maxVelocity / maxAcceleration;
    double accel_distance = 0.5 * maxAcceleration * acceleration_time * acceleration_time;
    if (2 * accel_distance > std::abs(distance)) {
        //triangle profile
        acceleration_time = std::sqrt(std::abs(distance) / maxAcceleration);
        cruise_time = 0.0;
        deceleration_time = acceleration_time;
    } else {
        //trapezoidal profile
        cruise_time = (std::abs(distance) - 2 * accel_distance) / maxVelocity;
        deceleration_time = acceleration_time;
    }
}

Vector3d TakeoffController::getTargetAcceleration(QCState& currentState, Pose3d currentPosition) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    //calculate target height based on motion profile
    double targetHeightAtTime;
    double targetAccelerationZ = 0.0;
    if (elapsed < acceleration_time) {
        targetHeightAtTime = 0.5 * maxAcceleration * elapsed * elapsed;
        targetAccelerationZ = maxAcceleration;
    } else if (elapsed < acceleration_time + cruise_time) {
        targetHeightAtTime = 0.5 * maxAcceleration * acceleration_time * acceleration_time
                                + maxVelocity * (elapsed - acceleration_time);
    } else if (elapsed < acceleration_time + cruise_time + deceleration_time) {
        double t = elapsed - (acceleration_time + cruise_time);
        targetHeightAtTime = 0.5 * maxAcceleration * acceleration_time * acceleration_time
                                + maxVelocity * cruise_time
                                + maxVelocity * t - 0.5 * maxAcceleration * t * t;
        targetAccelerationZ = -maxAcceleration;
    } else {
        targetHeightAtTime = targetHeight;
    }

    std::cout << "Target Height: " << targetHeightAtTime << std::endl;

    double heightError = targetHeightAtTime + currentPosition.getZ();
    Vector3d targetAcceleration = Vector3d{
        0.0,
        0.0,
        (targetAccelerationZ) + heightError * kP
    };
    return -targetAcceleration;
}