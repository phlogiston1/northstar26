#pragma once
#include <vector>
#include "Physics.h"

std::vector<double> getStateVector(State state);
std::vector<double> getStateVector(Pose3D position, Pose3D velocity);
std::vector<double> lqrControlStep(std::vector<double> current, std::vector<double> reference);
MotorVelocities applyMixer(std::vector<double> control_vector);