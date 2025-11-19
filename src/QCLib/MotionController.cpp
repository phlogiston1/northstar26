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
 * The motion controllers determine the target position and velocity to feed into the state space controller
 * to achieve the desired motion.
 *
 */

 //ManualController simply sets the motor velocities to a fixed value
VelocityController::VelocityController(double max_velocity, double max_acceleration, double max_jerk): max_velocity(max_velocity), max_acceleration(max_acceleration), max_jerk(max_jerk) {
}

void VelocityController::setInitialPose(State current) {
    pose = current.getPose().translation;
}

QCRequest VelocityController::getTarget(State current, Vector3D target_velocity) {
    Vector3D accel = (target_velocity - current.getLinearVelocity())/LOOP_TIME;
    if(accel.getMagnitude() > max_acceleration) accel = accel / (accel.getMagnitude() - max_acceleration);
    Vector3D velocity = current.getLinearVelocity() + (accel * LOOP_TIME);
    if(velocity.getMagnitude() > max_velocity) velocity = velocity /(velocity.getMagnitude() - max_velocity);

    velocity.z = target_velocity.z; //no accel limit on z velocity


    pose  = pose + (velocity * LOOP_TIME);

    return QCRequest(
        Pose3D(pose, Quaternion()),
        Pose3D(velocity, Quaternion())
    );
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
    auto next = path.sample(elapsed + LOOP_TIME);

    auto angle = getTargetAngle(current, Vector3D(sample.acc.x, sample.acc.y, 0));
    auto accel = getTargetAngle(current, Vector3D(next.acc.x, next.acc.y, 0));

    accel.rotateBy(angle.inverse());

    accel.print();

    return QCRequest(Pose3D(
        sample.pos.x,
        sample.pos.y,
        cruiseHeight,
        angle
    ), Pose3D(
        sample.vel.x,
        sample.vel.y,
        0,
        accel
    ));
}

QCRequest PathController::getTarget(State current, double yaw) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    std::cout << "DEBUG ELAPSED: " << elapsed << std::endl;
    auto sample = path.sample(elapsed);
    auto next = path.sample(elapsed + LOOP_TIME);

    auto angle = getTargetAngle(current, Vector3D(sample.acc.x, sample.acc.y, 0));
    auto accel = getTargetAngle(current, Vector3D(next.acc.x, next.acc.y, 0));
    angle.z = yaw;
    accel.z = yaw;

    accel.rotateBy(angle.inverse());

    accel.print();

    return QCRequest(Pose3D(
        sample.pos.x,
        sample.pos.y,
        cruiseHeight,
        angle
    ), Pose3D(
        sample.vel.x,
        sample.vel.y,
        0,
        accel
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

TakeoffController::TakeoffController(double kP, double velocity, double acceleration) : kP(kP), max_velocity(velocity), max_accel(acceleration) {
    setTargetHeight(0.0, 0.0);
}

void TakeoffController::setTargetHeight(double height, double current_height) {
    end_height = height;
    start_height = current_height;
    start_time = std::chrono::high_resolution_clock::now();
    //calculate motion profile times based on max acceleration and velocity
    double distance = end_height - current_height;
    accel_time = max_velocity / max_accel;
    double accel_distance = 0.5 * max_accel * accel_time * accel_time;
    if (2 * accel_distance > std::abs(distance)) {
        //triangle profile
        accel_time = std::sqrt(std::abs(distance) / max_accel);
        cruise_time = 0.0;
        deceleration_time = accel_time;
    } else {
        //trapezoidal profile
        cruise_time = (std::abs(distance) - 2 * accel_distance) / max_velocity;
        deceleration_time = accel_time;
    }
}

QCRequest TakeoffController::getTarget(State& currentState, Pose3D currentPosition) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time).count();
    //calculate target height based on motion profile
    double target_height;
    double target_vel;
    double target_accel = 0.0;

    //outrageous mess:
    if (elapsed < accel_time) {
        target_height = 0.5 * max_accel * elapsed * elapsed;
        target_vel = max_accel * elapsed;
        target_accel = max_accel;
    } else if (elapsed < accel_time + cruise_time) {
        target_height = 0.5 * max_accel * accel_time * accel_time
                                + max_velocity * (elapsed - accel_time);
        target_vel = max_velocity;
    } else if (elapsed < accel_time + cruise_time + deceleration_time) {
        double t = elapsed - (accel_time + cruise_time);
        target_height = 0.5 * max_accel * accel_time * accel_time
                                + max_velocity * cruise_time
                                + max_velocity * t - 0.5 * max_accel * t * t;
        target_vel = max_velocity - (elapsed - (accel_time + cruise_time))*max_accel;
        target_accel = -max_accel;
    } else {
        target_height = end_height;
        target_vel = 0;
    }

    double heightError = target_height + currentPosition.getZ();

    return QCRequest(
        Pose3D(0,0,target_height, Quaternion(M_PI_4/2,0,0)),
        Pose3D(0,0,target_vel)
    );
}