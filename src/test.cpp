#include "Quadcopter.h"
#include "InverseKinematics.h"
#include "Kinematics.h"
#include "Path.h"

#include <iostream>
#include <vector>

int main() {
    std::cout << "Welcome to the all inclusive QCLib test suite!" << std::endl;
    std::cout << std::endl;


    std::cout << "--------TEST: Kinematics sanity check--------" << std::endl;

    std::cout << "motor velocities fl:0 fr:0 rl:0 rr:0, should result in +Z acceleration due to gravity" << std::endl;
    MotorVelocities testVels(00.0, 00.0, 00.0, 00.0);
    Rotation3d testRotation = Rotation3d::fromDegrees(0.0, 0.0, 0.0);
    QCAcceleration accel = velocitiesToAccel(testVels, testRotation);
    std::cout << "\tAccel X: " << accel.getX() << " Y: " << accel.getY() << " Z: " << accel.getZ() <<  std::endl << "\tAngular Accel: ";
    accel.getAngular().print();

    std::cout << "motor velocities fl:2000 fr:2000 rl:2000 rr:2000, should result in -Z acceleration overcoming gravity" << std::endl;
    MotorVelocities testVels2(2000.0, 2000.0, 2000.0, 2000.0);
    QCAcceleration accel2 = velocitiesToAccel(testVels2, testRotation);
    std::cout << "\tAccel X: " << accel2.getX() << " Y: " << accel2.getY() << " Z: " << accel2.getZ() <<  std::endl << "\tAngular Accel: ";
    accel2.getAngular().print();

    std::cout << "motor velocities fl:2000 fr:2000 rl:1500 rr:1500, \n roll -45deg, should result in negative pitch accel (which is actually the copter angling forward) and -Z and Y axis acceleration" << std::endl;
    MotorVelocities testVels3(2000.0, 2000.0, 1500.0, 1500.0);
    Rotation3d testRotation3 = Rotation3d::fromDegrees(0.0, 0.0, -45.0);
    QCAcceleration accel3 = velocitiesToAccel(testVels3, testRotation3);
    std::cout << "\tAccel X: " << accel3.getX() << " Y: " << accel3.getY() << " Z: " << accel3.getZ() <<  std::endl << "\tAngular Accel: ";
    accel3.getAngular().print();


    std::cout << "\n\n--------TEST: Inverse Kinematics/Forward Kinematics consistency--------" << std::endl;
    MotorVelocities testVels4(1800.0, 1900.0, 2000.0, 2100.0);
    Rotation3d testRotation4 = Rotation3d::fromDegrees(10.0, -5.0, 15.0);
    QCAcceleration accel4 = velocitiesToAccel(testVels4, testRotation4);
    std::cout << "Given Motor Velocities fl:" << testVels4.getFrontLeft() << " fr:" << testVels4.getFrontRight()
              << " rl:" << testVels4.getRearLeft() << " rr:" << testVels4.getRearRight() << std::endl;
    std::cout << "\tCalculated Accel X: " << accel4.getX() << " Y: " << accel4.getY() << " Z: " << accel4.getZ() <<  std::endl << "\tAngular Accel: ";
    accel4.getAngular().print();
    TargetQCState tstate = calculateTargetState(
        QCState(Pose3d(), Pose3d(), testVels4, 0.0),
        Vector3d(accel4.getX(), accel4.getY(), accel4.getZ() + 9.8), //add gravity back in
        0.0 //no yaw rate
    );
    InverseKinematicResult ikResult = optimizeMotorVelocities(
        QCState(Pose3d(), Pose3d(), MotorVelocities(0,0,0,0), 0.0),
        tstate,
        0.001 //20ms timestep
    );
    std::cout << "Inverse Kinematics Resulting Motor Velocities fl:" << ikResult.motorVelocities.getFrontLeft() << " fr:" << ikResult.motorVelocities.getFrontRight()
              << " rl:" << ikResult.motorVelocities.getRearLeft() << " rr:" << ikResult.motorVelocities.getRearRight() << std::endl;
    std::cout << "note: these values will not match due to motor acceleration limits, and discretization of acceleration in InverseKinematics" << std::endl;

}