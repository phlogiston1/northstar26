#include "Quadcopter.h"
#include "Path.h"
#include "MotionController.h"
#include "LQR.h"
#include "Configuration.h"
#include <chrono> 


Quadcopter::Quadcopter(State initial, double max_velocity, double max_acceleration, double max_jerk):
    state(initial), 
    velocity_controller(VelocityController(max_velocity, max_acceleration, max_jerk)),
    path_controller(PathController()),
    height_controller(HeightController(max_velocity, max_acceleration)) {
        start_time = std::chrono::high_resolution_clock::now();
}

State Quadcopter::getState() {
    return state;
}

void Quadcopter::addVisionMeasurement(Vector3D translation, double timestamp) {

}

void Quadcopter::addIMUMeasurement(Quaternion angular_pos, Vector3D angular_vel) {
    state.pose.rotation = angular_pos;
    state.angular_velocity = angular_vel;
}

void Quadcopter::setHeight(double height) {
    height_controller.setTargetHeight(height, state.pose.getZ());
    manual = false;
}

void Quadcopter::beginPath(Path path) {
    // if(landing_status = LANDED) return;
    path_controller.beginPath(path, 0);
    manual = false;
}

void Quadcopter::beginManualControl(std::function<Vector3D()> velocity) {
    landing_status = FLYING;
    velocity_controller.setInitialPose(state);
    velocity_supplier = velocity;
    manual = true;
}

void Quadcopter::land() {
    landing_status = DECENT;
    setHeight(0.1);
}

bool Quadcopter::busy() {
    return !(height_controller.complete() && path_controller.complete());
}

bool Quadcopter::adjustingHeight() {
    return height_controller.complete();
}

MotorVelocities Quadcopter::getMotorVels() {
    return state.motor_velocities;
}

double Quadcopter::getTime() {
    auto time = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::seconds>(time - start_time).count();
}

void Quadcopter::update_simulation() {
    QCRequest req = QCRequest(Pose3D(), Pose3D());
    if(manual){
        req = velocity_controller.getTarget(state, velocity_supplier());
    } else {
        req = path_controller.getTarget(state);
        QCRequest height_request = height_controller.getTarget(state);

        req.position.translation.z = height_request.position.translation.z;
        req.velocity.translation.z = height_request.velocity.translation.z;
    }

    state.motor_velocities = applyMixer(lqrControlStep(
        getStateVector(state),
        getStateVector(req.position, req.velocity)
    ));

    state = state.predict(LOOP_TIME);
}