/**
 * @file InverseKinematics.cpp
 * @author Sean Benham
 * @brief Takes in a desired quadcopter acceleration and converts it to optimal motor velocities
 * @version 0.1
 * @date 2025-10-12
 * 
 * @copyright Copyright (c) 2025
 *
 */

#include "InverseKinematics.h"
#include "Quadcopter.h"
#include "Util.h"
#include "Configuration.h"
#include <cmath>
#include <iostream>

static double wrapPi(double a) {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

static Quaternion fromAxisAngle(const Vector3D& axis_in, double angle) {
    Vector3D axis = axis_in.normalized();
    double half = angle * 0.5;
    double s = std::sin(half);
    return Quaternion(std::cos(half), axis.x*s, axis.y*s, axis.z*s);
}

// Minimal rotation that sends a -> b
static Quaternion rotationFromAToB(const Vector3D& a_in, const Vector3D& b_in) {
    Vector3D a = a_in.normalized();
    Vector3D b = b_in.normalized();
    double cosTheta = a.dot(b);

    if (cosTheta > 1.0 - 1e-12) {
        return Quaternion(); // identity
    }
    if (cosTheta < -1.0 + 1e-12) {
        // 180° rotation: choose arbitrary perpendicular axis
        Vector3D ortho = Vector3D(1,0,0).cross(a);
        if (ortho.getMagnitude() < 1e-6) ortho = Vector3D(0,1,0).cross(a);
        ortho = ortho.normalized();
        return fromAxisAngle(ortho, M_PI);
    }

    Vector3D axis = a.cross(b).normalized();
    double angle = std::acos(std::max(-1.0, std::min(1.0, cosTheta)));
    return fromAxisAngle(axis, angle);
}




double thrustToVelocity(double thrust) {
    if(thrust < 0) return 0;
    return std::sqrt(std::abs(thrust) / THRUST_COEFF) * std::copysign(1.0, thrust);
}


TargetQCState calculateTargetState(State currentState, Vector3D targetAccel, double targetYawRate) {
    // Frame convention: +X forward, +Y right, +Z down (NED)
    //account for gravity and drag:
    targetAccel = targetAccel - Vector3D(0, 0, G);
    auto vel = currentState.getLinearVelocity();
    auto velocity_squared  = Vector3D(
        vel.x * std::abs(vel.x),
        vel.y * std::abs(vel.y),
        vel.z * std::abs(vel.z)
    );
    Vector3D dragForce = velocity_squared.componentWiseMultiply(Vector3D(LINEAR_DRAG_COEFF_XY, LINEAR_DRAG_COEFF_XY, LINEAR_DRAG_COEFF_Z));
    std::cout << "\ndrag: ";
    dragForce.print();
    
    targetAccel = targetAccel + (dragForce / QUADCOPTER_MASS);


    // 1. Get current yaw (assumed to be around Z axis)
    double fixed_yaw = currentState.getPose().rotation.getYaw();

    // 2. Normalize targetAccel to get direction (z_body axis)
    Vector3D z_body = -targetAccel.normalized(); // body Z axis in world frame (aligned with thrust direction)

    // 3. Compute desired x_c from yaw (projected on horizontal plane)
    double cy = cos(fixed_yaw);
    double sy = sin(fixed_yaw);
    Vector3D x_c(cy, sy, 0); // forward direction in world frame

    // 4. Compute orthogonal body axes using Gram-Schmidt process
    Vector3D y_body = z_body.cross(x_c);

    // Handle degenerate case if targetAccel is vertical (to avoid zero vector)
    if (y_body.getMagnitude() < 1e-6) {
        y_body = Vector3D(0, 1, 0); // pick arbitrary orthogonal vector
    }

    y_body = y_body.normalized();
    // Project x_c into plane orthogonal to z_body
    Vector3D x_body = x_c - z_body * x_c.dot(z_body);
    if (x_body.getMagnitude() < 1e-6) {
        // Degenerate case: choose any horizontal vector orthogonal to z_body
        x_body = Vector3D(1, 0, 0) - z_body * z_body.x;
    }

    x_body = x_body.normalized();

// Now y_body is guaranteed right-handed
    // 5. Build rotation from body axes
    Quaternion targetAngle = Quaternion::fromRotationMatrix(x_body, y_body, z_body);


    // To do this, we project the target acceleration onto the quadcopter's current Z axis.
    // double targetThrust = targetAccel.getZ() * QUADCOPTER_MASS;
    // Compute thrust (F = ma)
    // Rather than computing thrust based off of the target acceleration magnitude,
    // We want to compute the thrust based off of what is required to get the desired Z acceleration,
    // *At the quadcopters current orientation*,
    // since this will prevent the quadcopter from moving upward or downward while it is moving laterally.

    // To do this, we project the target acceleration onto the quadcopter's current Z axis.
    // Vector3d currentZAxis = currentState.getPose().rotation.getZAxis().normalized();
    Vector3D currentZAxis = targetAngle.getZAxis().normalized();

    double z_accel = -targetAccel.dot(currentZAxis);
    double targetThrust = z_accel * QUADCOPTER_MASS; //F=ma
    std::cout << "targetThrust: " << targetThrust << std::endl;

    if(targetThrust < 0) {
        targetThrust = 0;
    }

    return TargetQCState{targetAngle, targetThrust, targetYawRate};
}

InverseKinematicResult optimizeMotorVelocities(State currentState, TargetQCState targetState, double timestep) {
    //Step 1: Find the theretical forces at the points (width/2,0) and (0,width/2) that would produce the desired angular acceleration for pitch and roll.
    double desiredAngularVelPitch = (targetState.targetAngle.getPitch() - currentState.getPose().rotation.getPitch()) / timestep;
    double desiredAngularVelRoll = (targetState.targetAngle.getRoll() - currentState.getPose().rotation.getRoll()) / timestep;

    double desiredAngularAccelPitch = (desiredAngularVelPitch - currentState.getAngularVelocity().y) / timestep;
    double desiredAngularAccelRoll = (desiredAngularVelRoll - currentState.getAngularVelocity().x) / timestep;

    /*
    These values give us two formulas that give us the difference in thrust between opposite motors:
    tx = (rl - fr) - (rr - fl)
    ty = (rl - fr) + (rr - fl)

    Math note: we can find the drag torque about the yaw axis for a given thrust using:
    drag_torque = DRAG_COEFF * (thrust / THRUST_COEFF)
    or drag_torque = (DRAG_COEFF / THRUST_COEFF) * thrust

    since both drag torque and thrust are proportional to velocity squared
    */
    double torque_x = (desiredAngularAccelRoll * QUADCOPTER_MOI) / (QUADCOPTER_ROTOR_DISTANCE/2);
    double torque_y = (desiredAngularAccelPitch * QUADCOPTER_MOI) / (QUADCOPTER_ROTOR_DISTANCE/2);


    /*
    Step 2: Find the *equivalent thrust difference* needed to produce the desired yaw rate, from rotor drag torque=v^2 * DRAG_COEFF
    The reason I'm calculign them equivalent thrusts is so that we can just add them to the other thrust equations above.
    This gives the formula:
    equivalentThrustDifferenceYaw = (fl+rr) - (fr+rl) assuming front left rotor spins CW (negated for CCW)
    */
    double equivalentThrustDifferenceYaw = (targetState.targetYawRate * QUADCOPTER_MOI) / (ROTOR_DRAG_COEFF / THRUST_COEFF);
    if(FRONT_LEFT_SPINS_CCW) equivalentThrustDifferenceYaw = -equivalentThrustDifferenceYaw;


    /*
    our final formulas for the thrusts are:
    fl + fr + rl + rr = totalThrust
    (rl - fr) - (rr - fl) = torque_x
    (rl - fr) + (rr - fl) = torque_y
    (fl + rr) - (fr + rl) = equivalentThrustDifferenceYaw

    Solving these gives:
    fl = (totalThrust + torque_x - torque_y + equivalentThrustDifferenceYaw) / 4;
    fr = (totalThrust - torque_x - torque_y - equivalentThrustDifferenceYaw) / 4;
    rl = (totalThrust + torque_x + torque_y - equivalentThrustDifferenceYaw) / 4;
    rr = (totalThrust - torque_x + torque_y + equivalentThrustDifferenceYaw) / 4;
    */
    // std::cout << "\n\ntorque x: " << torque_x << "\n";
    double fl_adjustment = (+ torque_x - torque_y + equivalentThrustDifferenceYaw);
    double fr_adjustment = (- torque_x - torque_y - equivalentThrustDifferenceYaw);
    double rl_adjustment = (+ torque_x + torque_y - equivalentThrustDifferenceYaw);
    double rr_adjustment = (- torque_x + torque_y + equivalentThrustDifferenceYaw);

    // if(targetState.targetThrust + fl_adjustment < 0){

    // }


    double fl_thrust = (targetState.targetThrust + fl_adjustment) / 4.0;
    double fr_thrust = (targetState.targetThrust + fr_adjustment) / 4.0;
    double rl_thrust = (targetState.targetThrust + rl_adjustment) / 4.0;
    double rr_thrust = (targetState.targetThrust + rr_adjustment) / 4.0;

    double overdrive = 0;

    if(fl_thrust < 0) {
        overdrive = -fl_thrust;
        fl_thrust = 0;
    }
    if(fr_thrust < 0) {
        overdrive = std::max(overdrive, -fr_thrust);
    }
    if(rl_thrust < 0) {

    }

    //Step 3: Convert thrusts to velocities
    double fl_velocity = thrustToVelocity(fl_thrust);
    double fr_velocity = thrustToVelocity(fr_thrust);
    double rl_velocity = thrustToVelocity(rl_thrust);
    double rr_velocity = thrustToVelocity(rr_thrust);

    //Step 4: If motor acceleration exceeds the motor ramp rate, scale the difference between the current and target velocities accordingly.
    if(ENABLE_INV_KIN_MOTOR_CONTSTRAINTS) {
        double maxAllowedDeltaV = MOTOR_VELOCITY_RAMP_RATE * timestep;
        double fl_deltaV = fl_velocity - currentState.getMotorVelocities().getLeft();
        double fr_deltaV = fr_velocity - currentState.getMotorVelocities().getFront();
        double rl_deltaV = rl_velocity - currentState.getMotorVelocities().getRight();
        double rr_deltaV = rr_velocity - currentState.getMotorVelocities().getRear();
        double maxAbsDeltaV = std::max(std::max(std::abs(fl_deltaV), std::abs(fr_deltaV)), std::max(std::abs(rl_deltaV), std::abs(rr_deltaV)));
        if(maxAbsDeltaV > maxAllowedDeltaV) {
            double scale = maxAllowedDeltaV / maxAbsDeltaV;
            fl_velocity = currentState.getMotorVelocities().getLeft() + fl_deltaV * scale;
            fr_velocity = currentState.getMotorVelocities().getFront() + fr_deltaV * scale;
            rl_velocity = currentState.getMotorVelocities().getRight() + rl_deltaV * scale;
            rr_velocity = currentState.getMotorVelocities().getRear() + rr_deltaV * scale;
        }
    }

    //Step 5:

    return InverseKinematicResult{
        MotorVelocities{fl_velocity, fr_velocity, rl_velocity, rr_velocity},
        Acceleration(0,0,0,0,0,0), //TODO: calculate achieved acceleration
        0.0 //TODO: calculate error magnitude
    };
}

MotorVelocities optimizeMotorVelocities(State currentState, double thrust, double pitch_torque, double roll_torque, double yaw_torque) {
    double torque_x = roll_torque;
    double torque_y = pitch_torque;


    /*
    Step 2: Find the *equivalent thrust difference* needed to produce the desired yaw rate, from rotor drag torque=v^2 * DRAG_COEFF
    The reason I'm calculign them equivalent thrusts is so that we can just add them to the other thrust equations above.
    This gives the formula:
    equivalentThrustDifferenceYaw = (fl+rr) - (fr+rl) assuming front left rotor spins CW (negated for CCW)
    */
    double equivalentThrustDifferenceYaw = yaw_torque / (ROTOR_DRAG_COEFF / THRUST_COEFF);
    if(FRONT_LEFT_SPINS_CCW) equivalentThrustDifferenceYaw = -equivalentThrustDifferenceYaw;


    /*
    our final formulas for the thrusts are:
    fl + fr + rl + rr = totalThrust
    (rl - fr) - (rr - fl) = torque_x
    (rl - fr) + (rr - fl) = torque_y
    (fl + rr) - (fr + rl) = equivalentThrustDifferenceYaw

    Solving these gives:
    fl = (totalThrust + torque_x - torque_y + equivalentThrustDifferenceYaw) / 4;
    fr = (totalThrust - torque_x - torque_y - equivalentThrustDifferenceYaw) / 4;
    rl = (totalThrust + torque_x + torque_y - equivalentThrustDifferenceYaw) / 4;
    rr = (totalThrust - torque_x + torque_y + equivalentThrustDifferenceYaw) / 4;
    */
    // std::cout << "\n\ntorque x: " << torque_x << "\n";
    double fl_adjustment = (+ torque_x - torque_y + equivalentThrustDifferenceYaw);
    double fr_adjustment = (- torque_x - torque_y - equivalentThrustDifferenceYaw);
    double rl_adjustment = (+ torque_x + torque_y - equivalentThrustDifferenceYaw);
    double rr_adjustment = (- torque_x + torque_y + equivalentThrustDifferenceYaw);

    // if(targetState.targetThrust + fl_adjustment < 0){

    // }


    double fl_thrust = (thrust + fl_adjustment) / 4.0;
    double fr_thrust = (thrust + fr_adjustment) / 4.0;
    double rl_thrust = (thrust + rl_adjustment) / 4.0;
    double rr_thrust = (thrust + rr_adjustment) / 4.0;

    double overdrive = 0;

    if(fl_thrust < 0) {
        overdrive = -fl_thrust;
        fl_thrust = 0;
    }
    if(fr_thrust < 0) {
        overdrive = std::max(overdrive, -fr_thrust);
    }
    if(rl_thrust < 0) {

    }

    //Step 3: Convert thrusts to velocities
    double fl_velocity = thrustToVelocity(fl_thrust);
    double fr_velocity = thrustToVelocity(fr_thrust);
    double rl_velocity = thrustToVelocity(rl_thrust);
    double rr_velocity = thrustToVelocity(rr_thrust);

    //Step 4: If motor acceleration exceeds the motor ramp rate, scale the difference between the current and target velocities accordingly.
    if(ENABLE_INV_KIN_MOTOR_CONTSTRAINTS) {
        double maxAllowedDeltaV = MOTOR_VELOCITY_RAMP_RATE * LOOP_TIME;
        double fl_deltaV = fl_velocity - currentState.getMotorVelocities().getLeft();
        double fr_deltaV = fr_velocity - currentState.getMotorVelocities().getFront();
        double rl_deltaV = rl_velocity - currentState.getMotorVelocities().getRight();
        double rr_deltaV = rr_velocity - currentState.getMotorVelocities().getRear();
        double maxAbsDeltaV = std::max(std::max(std::abs(fl_deltaV), std::abs(fr_deltaV)), std::max(std::abs(rl_deltaV), std::abs(rr_deltaV)));
        if(maxAbsDeltaV > maxAllowedDeltaV) {
            double scale = maxAllowedDeltaV / maxAbsDeltaV;
            fl_velocity = currentState.getMotorVelocities().getLeft() + fl_deltaV * scale;
            fr_velocity = currentState.getMotorVelocities().getFront() + fr_deltaV * scale;
            rl_velocity = currentState.getMotorVelocities().getRight() + rl_deltaV * scale;
            rr_velocity = currentState.getMotorVelocities().getRear() + rr_deltaV * scale;
        }
    }

    //Step 5:

    return MotorVelocities{fl_velocity, fr_velocity, rl_velocity, rr_velocity};
}