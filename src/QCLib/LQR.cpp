#include "Util.h"
#include "Configuration.h"
#include "Quadcopter.h"

//GET USING compute_K.py
static const std::vector<std::vector<double>> LQR_K = {
{-0.000000,-0.000000,6.716873,-0.000000,-0.000000,6.571935,0.000000,-0.000000,-0.000000,0.000000,-0.000000,-0.000000},
{-0.000000,-1.309546,0.000000,0.000000,-1.638736,0.000000,6.204574,0.000000,0.000000,1.127978,0.000000,0.000000},
{1.309546,-0.000000,0.000000,1.638736,-0.000000,0.000000,0.000000,6.204574,-0.000000,0.000000,1.127978,-0.000000},
{-0.000000,-0.000000,0.000000,-0.000000,-0.000000,-0.000000,0.000000,-0.000000,3.126511,0.000000,-0.000000,1.978831},
};

//GET USING compute_mixer.py
static const std::vector<std::vector<double>> LQR_MIXER = {
{0.250000,3.535534,0.000000,-0.250000},
{0.250000,0.000000,3.535534,0.250000},
{0.250000,-3.535534,-0.000000,-0.250000},
{0.250000,-0.000000,-3.535534,0.250000},
};

double thrustToVel(double thrust) {
    // if(thrust < 0) return 0;
    return std::sqrt(std::abs(thrust) / THRUST_COEFF) * std::copysign(1.0, thrust);
}


std::vector<std::vector<double>> negateArray(const std::vector<std::vector<double>>& input) {
    std::vector<std::vector<double>> result(input.size(), std::vector<double>());

    for (size_t i = 0; i < input.size(); ++i) {
        result[i].reserve(input[i].size());
        for (double val : input[i]) {
            result[i].push_back(-val);
        }
    }

    return result;
}

std::vector<double> matrix_vector_multiply(std::vector<std::vector<double>> matrix, std::vector<double> vec) {
    if (matrix.empty() || vec.empty())
        throw std::invalid_argument("Matrix or vector cannot be empty.");

    size_t rows = matrix.size();
    size_t cols = matrix[0].size();

    if (vec.size() != cols)
        throw std::invalid_argument("Matrix columns must match vector size.");

    std::vector<double> result(rows, 0.0);

    for (size_t i = 0; i < rows; ++i) {
        if (matrix[i].size() != cols)
            throw std::invalid_argument("Matrix rows must all have the same number of columns.");
        for (size_t j = 0; j < cols; ++j) {
            result[i] += matrix[i][j] * vec[j];
        }
    }

    return result;
}

std::vector<double> subtract_vectors_elementwise(const std::vector<double>& v1, const std::vector<double>& v2) {
    if (v1.size() != v2.size()) {
        // Handle error: vectors must have the same size for element-wise subtraction
        // For simplicity, this example assumes equal sizes or you might throw an exception.
        throw std::invalid_argument("cannot subtract vectors of unequal length");
    }

    std::vector<double> result(v1.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        result[i] = v1[i] - v2[i];
    }
    return result;
}

std::vector<double> add_vectors_elementwise(const std::vector<double>& v1, const std::vector<double>& v2) {
    if (v1.size() != v2.size()) {
        // Handle error: vectors must have the same size for element-wise subtraction
        // For simplicity, this example assumes equal sizes or you might throw an exception.
        throw std::invalid_argument("cannot add vectors of unequal length");
    }

    std::vector<double> result(v1.size());
    for (size_t i = 0; i < v1.size(); ++i) {
        result[i] = v1[i] + v2[i];
    }
    return result;
}

/*
    NOTE: thrust order is in this coordinate frame:
         2Y
         |
     1---O---3X
         |
         4

    OURS:
    1 X 2
      X Y
    4   3

    Steps to convert:
    1. invert Z axis
    2. 
*/

// Pose3d convertPoseFrame(Pose3d original){
//     auto rotated = original.rotateBy(Rotation3d::fromDegrees(-45,0,0));
//     return Pose3d(
//         rotated.getY(),
//     )
// }

std::vector<double> getStateVector(State state) {
    // auto position_adj = state.getPose().rotateBy(Rotation3d::fromDegrees(45,0,180));
    // auto velocity_adj = state.getVelocity().rotateBy(Rotation3d::fromDegrees(45,0,180));
    auto position_adj = state.getPose();
    auto velocity_adj = state.getLinearVelocity();

    auto pos = position_adj.translation;
    auto vel = state.getLinearVelocity();
    auto ang_pos = position_adj.rotation;
    auto ang_vel = state.getAngularVelocity();
    return {
        pos.x, pos.y, pos.z,
        vel.x, vel.y, vel.z,
        ang_pos.getRoll(), ang_pos.getPitch(), ang_pos.getYaw(),
        ang_vel.x, ang_vel.y, ang_vel.z
    };
}

std::vector<double> getStateVector(Pose3D position, Pose3D velocity) {
    // auto position_adj = position.rotateBy(Rotation3d::fromDegrees(45,0,180));
    // auto velocity_adj = velocity.rotateBy(Rotation3d::fromDegrees(45,0,180));
     auto position_adj = position;
    auto velocity_adj = velocity;

    auto pos = position_adj.translation;
    auto vel = velocity_adj.translation;
    auto ang_pos = position_adj.rotation;
    auto ang_vel = velocity_adj.rotation;
    return {
        pos.x, pos.y, pos.z,
        vel.x, vel.y, vel.z,
        ang_pos.getRoll(), ang_pos.getPitch(), ang_pos.getYaw(),
        ang_vel.getRoll(), ang_vel.getPitch(), ang_vel.getYaw()
    };
}

double angle_limit = M_PI/4;

/**
 * @brief Get the output of an LQR control step.
 * 
 * @param current vector countaining: {position x, position y, position z, velocity x, velocity y, velocity z, roll, pitch, yaw, roll rate, pitch rate, yaw rate}
 * @param reference vector countaining: {position x, position y, position z, velocity x, velocity y, velocity z, roll, pitch, yaw, roll rate, pitch rate, yaw rate}
 * @return std::vector<double> containing: {total force (thrust), roll torque, pitch torque, yaw torque}
 */
std::vector<double> lqrControlStep(std::vector<double> current, std::vector<double> reference) {

    // if(current[8] > angle_limit) current[8] = angle_limit;
    // if(current[8] < -angle_limit) current[8] = -angle_limit;
    // if(current[7] > angle_limit) current[7] = angle_limit;
    // if(current[7] < -angle_limit) current[7] = -angle_limit;

    std::vector<double> error = subtract_vectors_elementwise(current, reference);
    Pose3D positionError = Pose3D(error[0], error[1], error[2], Quaternion(error[8], error[7], error[6]));
    Pose3D velocityError = Pose3D(error[3], error[4], error[5], Quaternion(error[11], error[10], error[9]));


    //rotate the error by the quadcopter's current yaw to convert to local coordinates
    positionError = positionError.rotateBy(Quaternion(-current[8],0,0));
    velocityError = velocityError.rotateBy(Quaternion(-current[8],0,0));

    std::vector<double> error_local  = getStateVector(positionError, velocityError);
    //bring the original yaw / yaw rate into the local coordinate vector since it doesn't need to change.
    error_local[8] = error[8];
    error_local[11] = error[11];


    std::vector<double> feedback = matrix_vector_multiply(negateArray(LQR_K), error_local);

    std::vector<double> steady_state = {
        QUADCOPTER_MASS * G,// * cos(current[7]) * cos(current[8]),
        0,
        0,
        0
    };

    return add_vectors_elementwise(feedback, steady_state);
}

MotorVelocities applyMixer(std::vector<double> control_vector) {
    //gives {front, left, back, right}
    auto thrusts = matrix_vector_multiply(LQR_MIXER, control_vector);

    /*
    front = 0
    left = 1
    back = 2
    right = 3
    */


    return MotorVelocities(
        thrustToVel(thrusts[1]),
        thrustToVel(thrusts[0]),
        thrustToVel(thrusts[3]),
        thrustToVel(thrusts[2])
    );
}