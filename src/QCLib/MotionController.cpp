#include "MotionController.h"
#include "Configuration.h"
#include "Util.h"
#include <cmath>
/*
NOTE: BUGS TO FIX:
- double distance bug in this file
- starting position not accounted bug in this file
- requestiong negative motor velocity causing infinite motor speed blowup bug (somewhere)
*/


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

Vector3D VelocityController::getTargetAcceleration(State& currentState, const Vector3D& targetVelocity) {
    Vector3D targetAcceleration = targetVelocity - currentState.getLinearVelocity();
    //limit acceleration based on max acceleration constants
    //it would be theoretically better to use polar coordinates here, but this is less computationally intensive
    if (targetAcceleration.x > MAX_ACCELERATION_XY) targetAcceleration.x = MAX_ACCELERATION_XY;
    if (targetAcceleration.x < -MAX_ACCELERATION_XY) targetAcceleration.x = -MAX_ACCELERATION_XY;
    if (targetAcceleration.y > MAX_ACCELERATION_XY) targetAcceleration.y = MAX_ACCELERATION_XY;
    if (targetAcceleration.y < -MAX_ACCELERATION_XY) targetAcceleration.y = -MAX_ACCELERATION_XY;
    if (targetAcceleration.z > MAX_ACCELERATION_Z) targetAcceleration.z = MAX_ACCELERATION_Z;
    if (targetAcceleration.z < -MAX_ACCELERATION_Z) targetAcceleration.z = -MAX_ACCELERATION_Z;
    //smooth acceleration changes based on max jerk constants. Z axis doesn't need jerk constraint.
    Vector3D jerk = targetAcceleration - lastTargetAcceleration;
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

QCRequest PathController::getTarget(State current) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    std::cout << "DEBUG ELAPSED: " << elapsed << std::endl;
    auto sample = path.sample(elapsed);

    auto angle = calculateTargetState(current, Vector3D(sample.acc.x, sample.acc.y, 0), 0);
    angle.targetAngle.z = sample.pos.x/5;

    return QCRequest(Pose3D(
        sample.pos.x,
        sample.pos.y,
        cruiseHeight,
        angle.targetAngle
    ), Pose3D(
        sample.vel.x,// * 0.5,
        sample.vel.y,// * 0.5,
        0
    ));
}

Vector3D PathController::getTargetAcceleration(State& currentState, Pose3D currentPosition) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    PathPoint targetPoint = path.sample(elapsed);
    Vector2D positionError = targetPoint.pos - currentPosition.translation.toVector2D();
    Vector2D velocityError = targetPoint.vel - currentState.getLinearVelocity().toVector2D();

    //modify velocity error based on position error and position kp
    velocityError = velocityError + Vector2D{
        positionError.x * position_kp.x,
        positionError.y * position_kp.y
    };

    Vector3D targetAcceleration = Vector3D{
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

Vector3D TakeoffController::getTargetAcceleration(State& currentState, Pose3D currentPosition) {
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
    Vector3D targetAcceleration = Vector3D{
        0.0,
        0.0,
        (targetAccelerationZ) + heightError * kP
    };
    return -targetAcceleration;
}

QCRequest TakeoffController::getTarget(State& currentState, Pose3D currentPosition) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    //calculate target height based on motion profile
    double targetHeightAtTime;
    double targetVelocityAtTime;
    double targetAccelerationZ = 0.0;
    if (elapsed < acceleration_time) {
        targetHeightAtTime = 0.5 * maxAcceleration * elapsed * elapsed;
        targetVelocityAtTime = maxAcceleration*elapsed;
        targetAccelerationZ = maxAcceleration;
    } else if (elapsed < acceleration_time + cruise_time) {
        targetHeightAtTime = 0.5 * maxAcceleration * acceleration_time * acceleration_time
                                + maxVelocity * (elapsed - acceleration_time);
        targetVelocityAtTime = maxVelocity;
    } else if (elapsed < acceleration_time + cruise_time + deceleration_time) {
        double t = elapsed - (acceleration_time + cruise_time);
        targetHeightAtTime = 0.5 * maxAcceleration * acceleration_time * acceleration_time
                                + maxVelocity * cruise_time
                                + maxVelocity * t - 0.5 * maxAcceleration * t * t;
        targetVelocityAtTime = maxVelocity - (elapsed - (acceleration_time + cruise_time))*maxAcceleration;
        targetAccelerationZ = -maxAcceleration;
    } else {
        targetHeightAtTime = targetHeight;
        targetVelocityAtTime = 0;
    }

    std::cout << "Target Vel: " << targetVelocityAtTime << std::endl;

    double heightError = targetHeightAtTime + currentPosition.getZ();

    return QCRequest(
        Pose3D(0,0,targetHeightAtTime),
        Pose3D(0,0,targetVelocityAtTime*0.5)//-targetVelocityAtTime)
    );
}