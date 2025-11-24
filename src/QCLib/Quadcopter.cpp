#include "Quadcopter.h"
#include "Path.h"
#include "MotionController.h"
#include "LQR.h"
#include "Configuration.h"
#include <chrono> 


Quadcopter::Quadcopter(State initial):
    state(initial), 
    velocity_controller(VelocityController()),
    path_controller(PathController()),
    height_controller(HeightController()) {
        start_time = std::chrono::high_resolution_clock::now();
}

State Quadcopter::getState() {
    return state;
}

void Quadcopter::addVisionMeasurement(Vector3D translation, double timestamp) {
    //TODO make this not sucky and stupid
    state.pose.translation = translation;
}

void Quadcopter::addIMUMeasurement(Quaternion angular_pos, Vector3D angular_vel) {
    state.pose.rotation = angular_pos;
    state.angular_velocity = angular_vel;
}

void Quadcopter::setHeight(double height) {
    height_controller.setTargetHeight(height, state.pose.getZ());
    manual = false;
}

void Quadcopter::addWaypoint(Vector2D waypoint) {
    path_waypoints.push_back(waypoint);
}

void Quadcopter::beginPath() {
    beginPath(Path(path_waypoints, MAX_VELOCITY_XY, MAX_ACCELERATION_XY, MAX_JERK_XY));
}

void Quadcopter::beginPath(Path path) {
    // if(landing_status = LANDED) return;
    path_controller.beginPath(path, 0);
    manual = false;
}

void Quadcopter::beginManualControl() {
    velocity_controller.setInitialPose(state);
    manual = true;
}

void Quadcopter::setVelocity(Vector3D velocity) {
    manual_velocity = velocity;
}

void Quadcopter::land() {
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

QCRequest* Quadcopter::getRequest() {
    if(manual){
        req = velocity_controller.getTarget(state, manual_velocity);
    } else {
        req = path_controller.getTarget(state);
        QCRequest height_request = height_controller.getTarget(state);

        req.position.translation.z = height_request.position.translation.z;
        req.velocity.translation.z = height_request.velocity.translation.z;
    }
    return &req;
}

Vector3D* Quadcopter::getTranslation() {
    return &state.pose.translation;
}

Vector3D* Quadcopter::getVelocity() {
    return &state.linear_velocity;
}

void Quadcopter::updateSimulation() {
    // getRequest();

    // state.motor_velocities = applyMixer(lqrControlStep(
    //     getStateVector(state),
    //     getStateVector(req.position, req.velocity)
    // ));

    state = state.fullKinematicsStep(LOOP_TIME);
}

void Quadcopter::updateKinematics() {
    state = state.translationKinematicsStep(LOOP_TIME);
}

void Quadcopter::setMotorVelocities(MotorVelocities vels) {
    state.motor_velocities = vels;
}

bool Quadcopter::isManual() {
    return manual;
}