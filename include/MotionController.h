#include "Quadcopter.h"
#include "Path.h"
#include "InverseKinematics.h"
#include <chrono>

struct QCRequest{
    Pose3D position, velocity;
    QCRequest(Pose3D position, Pose3D velocity): position(position), velocity(velocity) {

    }
};

class VelocityController{
    public:
        VelocityController(double max_velocity, double max_acceleration, double max_jerk);
        void setInitialPose(State current);
        QCRequest getTarget(State current, Vector3D target_velocity);
    private:
        Vector3D velocity = Vector3D(0,0,0);
        Vector3D pose = Vector3D(0,0,0);
        double max_velocity, max_acceleration, max_jerk;
};

class PathController{
    public:
        PathController(Vector2D position_kp, Vector2D velocity_kp, double cruiseHeight_kP);
        void beginPath(const Path& newPath, double cruiseHeight);
        QCRequest getTarget(State current);
        QCRequest getTarget(State current, double yaw);
        Vector3D getTargetAcceleration(State& currentState, Pose3D currentPosition);
    private:
        Path path;
        double cruiseHeight;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
        Vector2D position_kp;
        Vector2D velocity_kp;
        double cruiseHeight_kP;
};

//uses trapeoidal motion profile to smoothly take off to a target height
class TakeoffController{
    public:
        TakeoffController(double kP, double maxVelocity, double maxAcceleration);
        void setTargetHeight(double height, double currentHeight);
        Vector3D getTargetAcceleration(State& currentState, Pose3D currentPosition);
        QCRequest getTarget(State& currentState, Pose3D currentPosition);
    private:
        double start_height;
        double end_height;
        double max_velocity;
        double max_accel;
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
        double accel_time; //time spent accelerating
        double cruise_time; //time spent at constant velocity
        double deceleration_time; //time spent decelerating
        double kP;
};