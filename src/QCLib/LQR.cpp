#include "Util.h"
#include "Configuration.h"

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

std::vector<double> lqrControlStep(std::vector<double> current, std::vector<double> reference) {
    std::vector<double> error = subtract_vectors_elementwise(current, reference);
    std::vector<double> feedback = matrix_vector_multiply(negateArray(LQR_K), error);

    std::vector<double> steady_state = {
        QUADCOPTER_MASS * G,
        0,
        0,
        0
    };

    return add_vectors_elementwise(feedback, steady_state);
}