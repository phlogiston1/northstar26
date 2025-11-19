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


    std::cout << "\n\n\n\n\n\n LQR INITIAL TESTING:" << std::endl;
    State current = State(
        Pose3D(0,0,0,Quaternion(0,0,0)),
        Vector3D(0,0,0),
        Vector3D(0,0,0),
        MotorVelocities(0,0,0,0),
        0
    );
    State reference = State(
        Pose3D(0,0,0,Quaternion(0,0,0)),
        Vector3D(0,0,0),
        Vector3D(0,0,0),
        MotorVelocities(0,0,0,0),
        0
    );
    auto result = lqrControlStep(getStateVector(current), getStateVector(reference));
    std::cout << "thrust: " << result[0] << std::endl;
    std::cout << "roll torque: " << result[1] << std::endl;
    std::cout << "pitch torque: " << result[2] << std::endl;
    std::cout << "yaw torque: " << result[3] << std::endl;

    MotorVelocities motorvels = applyMixer(result);
    std::cout << "Motor Velocities Left:" << motorvels.getLeft() << " Front:" << motorvels.getFront()
              << " Right:" << motorvels.getRight() << " Rear:" << motorvels.getRear() << std::endl;

    std::cout << "\n\nKINEMATICS ACCELERATION: \n";
    current.setMotorVelocities(motorvels);
    auto acc = velocitiesToAccel(current);
    std::cout << "\tCalculated Accel X: " << acc.getX() << " Y: " << acc.getY() << " Z: " << acc.getZ() <<  std::endl << "\tAngular Accel: ";
    std::cout << "Yaw: " << acc.getYaw() << " Pitch: " << acc.getPitch() << " Roll: " << acc.getRoll();

    // Pose3d(0,0,1).rotateBy(Rotation3d::fromDegrees(45,0,180)).print();

    // TakeoffController test = TakeoffController(0,1,5);

    // test.setTargetHeight(5,0);

    // for(double i = 0; i < 3; i+=0.1) {
    //     test.getTarget(current, current.getPose());
    // }

    auto path = Path(
        {
            Vector2D{0,0},
            Vector2D{1,2},
            Vector2D{3,3}
        },
        1, // max velocity
        1, // max acceleration
        1  // max jerk
    );

    double dt = 0.1;
    std::cout << "\nPath Sampling:\n";
    for(double t = 0.0; t <= path.getTotalTime(); t += dt) {
        PathPoint pt = path.sample(t);
        std::cout << "t=" << t << ": Pos(" << pt.pos.x << ", " << pt.pos.y << "), Vel(" << pt.vel.x << ", " << pt.vel.y << "), Acc(" << pt.acc.x << ", " << pt.acc.y << ")\n";
    }

}//Motor Velocities fl:648.405 fr:0 rl:1649.46 rr:648.405