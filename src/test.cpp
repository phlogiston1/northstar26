#include "Quadcopter.h"
#include "InverseKinematics.h"
#include "Kinematics.h"
#include "Path.h"

#include <iostream>
#include <vector>

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

    //test path generation with util.h

    auto path = Path(
        {
            Vector2D{0,0},
            Vector2D{1,2},
            Vector2D{3,3},
            Vector2D{4,0}
        },
        50, // max velocity
        2.0, // max acceleration
        5.0  // max jerk
    );

    double dt = 0.1;
    std::cout << "\nPath Sampling:\n";
    for(double t = 0.0; t <= path.getTotalTime(); t += dt) {
        PathPoint pt = path.sample(t);
        std::cout << "t=" << t << ": Pos(" << pt.pos.x << ", " << pt.pos.y << "), Vel(" << pt.vel.x << ", " << pt.vel.y << "), Acc(" << pt.acc.x << ", " << pt.acc.y << ")\n";
    }
}