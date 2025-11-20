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
#include "Physics.h"
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


Quaternion getTargetAngle(State currentState, Vector3D targetAccel) {
    // Frame convention: +X forward, +Y right, +Z down (NED)
    //account for gravity and drag:
    targetAccel = targetAccel + Vector3D(0, 0, G);

    auto vel = currentState.linear_velocity;
    auto velocity_squared  = Vector3D(
        vel.x * std::abs(vel.x),
        vel.y * std::abs(vel.y),
        vel.z * std::abs(vel.z)
    );
    Vector3D dragForce = velocity_squared.componentWiseMultiply(Vector3D(LINEAR_DRAG_COEFF_XY, LINEAR_DRAG_COEFF_XY, LINEAR_DRAG_COEFF_Z));


    targetAccel = targetAccel + (dragForce / QUADCOPTER_MASS);


    // 1. Get current yaw (assumed to be around Z axis)
    double fixed_yaw = currentState.pose.rotation.getYaw();

    // 2. Normalize targetAccel to get direction (z_body axis)
    Vector3D z_body = targetAccel.normalized(); // body Z axis in world frame (aligned with thrust direction)

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

    Quaternion targetAngle = Quaternion::fromRotationMatrix(x_body, y_body, z_body);


    return targetAngle;
}