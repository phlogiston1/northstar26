#include "Quadcopter.h"
#include "Path.h"
#include "InverseKinematics.h"
#include <chrono>

class VelocityController{
    public:
        VelocityController();
        Vector3D getTargetAcceleration(QCState& currentState, const Vector3D& targetVelocity);
    private:
        Vector3D lastTargetAcceleration = Vector3D(0,0,0);
};

class PathController{
    public:
        PathController(Vector2D position_kp, Vector2D velocity_kp, double cruiseHeight_kP);
        void beginPath(const Path& newPath, double cruiseHeight);
        Vector3D getTargetAcceleration(QCState& currentState, Pose3d currentPosition);
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
        Vector3D getTargetAcceleration(QCState& currentState, Pose3d currentPosition);
    private:
        double targetHeight;
        double maxVelocity;
        double maxAcceleration;
        std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
        double acceleration_time; //time spent accelerating
        double cruise_time; //time spent at constant velocity
        double deceleration_time; //time spent decelerating
        double kP;
};