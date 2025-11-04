#include "Quadcopter.h"
#include "InverseKinematics.h"
#include "Kinematics.h"

#include <iostream>

int main() {
    QCState currentState = QCState(
        Pose3d(Vector3d(0,0,0), Rotation3d::fromDegrees(0,5,0)),
        Pose3d(Vector3d(0,0,0), Rotation3d::fromDegrees(0,0,0)),
        MotorVelocities(1331,1331,1331,1331),
        0
    );

    std::cout << "Accel from defined current state:\n";
    QCAcceleration currentAccel = velocitiesToAccel(currentState.getMotorVelocities(), currentState.getPose().rotation);
    currentAccel.getAngular().print();
    std::cout << "X: " << currentAccel.getX() << " Y: "<< currentAccel.getY() << " Z: " << currentAccel.getZ() << std::endl;

    Vector3d targetAccel(0,0,1);
    double targetYawRate = 0.05;

    

    TargetQCState targetState = calculateTargetState(currentState, targetAccel, targetYawRate);

    std::cout << "\nIdeal State Rotation: " << std::endl;
    targetState.targetAngle.print();


    std::cout << "Ideal State Thrust:" <<targetState.targetThrust << std::endl;

    InverseKinematicResult ikResult = optimizeMotorVelocities(currentState, targetState, 1);
    std::cout << "\nInverse Kinematics Motor Velocities:\n\tFront Left: "
              << ikResult.motorVelocities.getFrontLeft() << ", \n\tFront Right: "
              << ikResult.motorVelocities.getFrontRight() << ", \n\tRear Left: "
              << ikResult.motorVelocities.getRearLeft() << ", \n\tRear Right: "
              << ikResult.motorVelocities.getRearRight() << std::endl;

    auto accel = velocitiesToAccel(ikResult.motorVelocities, targetState.targetAngle);
    std::cout << "\nAchieved Accel: " << std::endl;
    accel.getAngular().print();
    std::cout << "X: " << accel.getX() << " Y: "<< accel.getY() << " Z: " << accel.getZ() << std::endl;
}