#include "utils_test.h"
#include <iostream>


std::vector<std::vector<double>> readMatrixFromCSV(const std::string& filename) {
    std::vector<std::vector<double>> matrix;
    std::ifstream file(filename);

    // Check if the file is successfully opened
    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);

        double value;
        while (ss >> value) {
            row.push_back(value);
            if (ss.peek() == ',') ss.ignore();
        }
        matrix.push_back(row);
    }

    return matrix;
}

bool compareMatrices(const gsl_matrix* A, const std::vector<std::vector<double>>& B, double tolerance) {
    if (A->size1 != B.size() || A->size2 != B[0].size()){
        std::cout << "Size mismatch: " << std::endl;
        std::cout << "Matrix A: " << A->size1 << "x" << A->size2 << std::endl;
        std::cout << "Matrix B: " << B.size() << "x" << B[0].size() << std::endl;
        return false;
    }

    bool state = true;
    for (size_t i = 0; i < A->size1; ++i) {
        for (size_t j = 0; j < A->size2; ++j) {
            if (std::fabs(gsl_matrix_get(A, i, j) - B[i][j]) > tolerance) {
                std::cout << "Mismatch at (" << i << ", " << j << "): " << gsl_matrix_get(A, i, j) << " != " << B[i][j] << std::endl;
                state = false;
            }
        }
    }
    return state;
}
