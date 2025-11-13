#include "Quadcopter.h"
#include "InverseKinematics.h"
#include "Kinematics.h"
#include "Path.h"
#include "LQR.h"
#include "MotionController.h"

#include <iostream>
#include <vector>

int main() {
    std::cout << "Welcome to the all inclusive QCLib test suite!" << std::endl;
    std::cout << std::endl;


    std::cout << "--------TEST: Kinematics sanity check--------" << std::endl;

    // std::cout << "motor velocities fl:0 fr:0 rl:0 rr:0, should result in +Z acceleration due to gravity" << std::endl;
    // QCState initialState = QCState(Pose3d(), Pose3d(), MotorVelocities(0,0,0,0), 0);
    // QCAcceleration accel = velocitiesToAccel(initialState);
    // std::cout << "\tAccel X: " << accel.getX() << " Y: " << accel.getY() << " Z: " << accel.getZ() <<  std::endl << "\tAngular Accel: ";
    // accel.getAngular().print();

    // std::cout << "motor velocities fl:2000 fr:2000 rl:2000 rr:2000, should result in -Z acceleration overcoming gravity" << std::endl;
    // initialState.setMotorVelocities(MotorVelocities(2000.0, 2000.0, 2000.0, 2000.0));
    // QCAcceleration accel2 = velocitiesToAccel(initialState);
    // std::cout << "\tAccel X: " << accel2.getX() << " Y: " << accel2.getY() << " Z: " << accel2.getZ() <<  std::endl << "\tAngular Accel: ";
    // accel2.getAngular().print();

    // std::cout << "motor velocities fl:2000 fr:2000 rl:1500 rr:1500, \n roll -45deg, should result in negative pitch accel (which is actually the copter angling forward) and -Z and Y axis acceleration" << std::endl;
    // MotorVelocities testVels3(2000.0, 2000.0, 1500.0, 1500.0);
    // Rotation3d testRotation3 = Rotation3d::fromDegrees(0.0, 0.0, -45.0);
    // QCAcceleration accel3 = velocitiesToAccel(testVels3, testRotation3);
    // std::cout << "\tAccel X: " << accel3.getX() << " Y: " << accel3.getY() << " Z: " << accel3.getZ() <<  std::endl << "\tAngular Accel: ";
    // accel3.getAngular().print();


    std::cout << "\n\n--------TEST: Inverse Kinematics/Forward Kinematics consistency--------" << std::endl;
    QCState initialState = QCState(
        Pose3d(),
        Pose3d(0,0,0.25),
        MotorVelocities(2000,2000,2000,2000), 
        0);

    // Rotation3d testRotation4 = Rotation3d::fromDegrees(0.0, 0.0, 0.0);
    QCAcceleration accel4 = velocitiesToAccel(initialState);
    // std::cout << "Given Motor Velocities fl:" << testVels4.getFrontLeft() << " fr:" << testVels4.getFrontRight()
            //   << " rl:" << testVels4.getRearLeft() << " rr:" << testVels4.getRearRight() << std::endl;
    std::cout << "\tCalculated Accel X: " << accel4.getX() << " Y: " << accel4.getY() << " Z: " << accel4.getZ() <<  std::endl << "\tAngular Accel: ";
    accel4.getAngular().print();
    TargetQCState tstate = calculateTargetState(
        initialState,
        Vector3D(accel4.getX(),accel4.getY(),accel4.getZ()),
        // Vector3d(0,0,-1),
        0.0 //no yaw rate
    );
    std::cout << "target thrust: " << tstate.targetThrust << "\ntarget angle: ";
    tstate.targetAngle.print();

    InverseKinematicResult ikResult = optimizeMotorVelocities(
        QCState(Pose3d(), Pose3d(), MotorVelocities(0,0,0,0), 0.0),
        tstate,
        0.1 //20ms timestep
    );
    std::cout << "Inverse Kinematics Resulting Motor Velocities fl:" << ikResult.motorVelocities.getFrontLeft() << " fr:" << ikResult.motorVelocities.getFrontRight()
              << " rl:" << ikResult.motorVelocities.getRearLeft() << " rr:" << ikResult.motorVelocities.getRearRight() << std::endl;
    std::cout << "note: these values may not match exactly due to motor acceleration limits, and discretization of acceleration in InverseKinematics" << std::endl;





    std::cout << "\n\n\n\n\n\n LQR INITIAL TESTING:" << std::endl;
    QCState current = QCState(
        Pose3d(0,0,0,Rotation3d(0,0,0)),
        Pose3d(0,0,0,Rotation3d(0,0,0)),
        MotorVelocities(0,0,0,0),
        0
    );
    QCState reference = QCState(
        Pose3d(0,0,-1,Rotation3d(0,0,0)),
        Pose3d(0,0,0,Rotation3d(0,0,0)),
        MotorVelocities(0,0,0,0),
        0
    );
    auto result = lqrControlStep(getStateVector(current), getStateVector(reference));
    std::cout << "thrust: " << result[0] << std::endl;
    std::cout << "pitch torque: " << result[1] << std::endl;
    std::cout << "roll torque: " << result[2] << std::endl;
    std::cout << "yaw torque: " << result[3] << std::endl;

    auto motorvels = optimizeMotorVelocities(current, result[0], result[1], result[2], result[3]);
    // std::cout << "Motor Velocities fl:" << motorvels.getFrontLeft() << " fr:" << motorvels.getFrontRight()
    //           << " rl:" << motorvels.getRearLeft() << " rr:" << motorvels.getRearRight() << std::endl;
    motorvels = applyMixer(result);
    std::cout << "Motor Velocities fl:" << motorvels.getFrontLeft() << " fr:" << motorvels.getFrontRight()
              << " rl:" << motorvels.getRearLeft() << " rr:" << motorvels.getRearRight() << std::endl;

    Pose3d(0,0,1).rotateBy(Rotation3d::fromDegrees(45,0,180)).print();

    TakeoffController test = TakeoffController(0,1,5);

    test.setTargetHeight(5,0);

    for(double i = 0; i < 3; i+=0.1) {
        test.getTarget(current, current.getPose());
    }

}//Motor Velocities fl:648.405 fr:0 rl:1649.46 rr:648.405