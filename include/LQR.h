#pragma once
#include <vector>
#include "Quadcopter.h"

std::vector<double> getStateVector(QCState state);
std::vector<double> getStateVector(Pose3d position, Pose3d velocity);
std::vector<double> lqrControlStep(std::vector<double> current, std::vector<double> reference);
MotorVelocities applyMixer(std::vector<double> control_vector);