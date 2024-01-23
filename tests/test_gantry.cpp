#include "gtest/gtest.h"
#include "gantry.h"
#include "utils_test.h"
#include <gsl/gsl_matrix.h>
#define PATH_TO_MATRIX_0 "../../tests/Matrix/test_matrix_0.csv"
#define PATH_TO_MATRIX_1 "../../tests/Matrix/test_matrix_1.csv"

TEST(ModelTest, CompareMatrixSizes) {
    const size_t size = 11;
    double x[size] = {0}; // Adjust as needed for your test
    gsl_matrix* A = gsl_matrix_alloc(size, size);
    auto B = readMatrixFromCSV(PATH_TO_MATRIX_0); // Read matrix from CSV"); 

    gantry::get_matrix_A(A, x); // Populate A using your function
    size_t rows_A = A->size1; 
    size_t cols_A = A->size2; 
    size_t rows_B = B.size();     // Assuming matrix_A is a gsl_matrix* 
    size_t cols_B = B[0].size(); 

    // Compare sizes
    if (rows_A != rows_B || cols_A != cols_B) {
        // If sizes don't match, print the sizes and fail the test
        std::cout << "Size mismatch: " << std::endl;
        std::cout << "Matrix A: " << rows_A << "x" << cols_A << std::endl;
        std::cout << "Matrix B: " << rows_B << "x" << cols_B << std::endl;
        FAIL() << "Matrix sizes do not match.";
    } else {
        SUCCEED() << "Matrix sizes match.";
    }
}

TEST(ModelTest, ZeroInputCheck) {
    const size_t size = 11;
    double x[size] = {0}; // Adjust as needed for your test
    gsl_matrix* A = gsl_matrix_alloc(size, size);

    gantry::get_matrix_A(A, x); // Populate A using your function

    auto matrixFromCSV = readMatrixFromCSV(PATH_TO_MATRIX_0); // Read matrix from CSV

    EXPECT_TRUE(compareMatrices(A, matrixFromCSV, 1e-3)); // Compare matrices

    gsl_matrix_free(A); // Clean up
}

TEST(ModelTest, PseudoRandomInputCheck) {
    const size_t size = 11;
    double x[size] = {30.0,1005.142142,31.12312,23,23,0.234234,0.24234,100,1,5,0};
    gsl_matrix* A = gsl_matrix_alloc(size, size);

    gantry::get_matrix_A(A, x);

    auto matrixFromCSV = readMatrixFromCSV(PATH_TO_MATRIX_1);

    EXPECT_TRUE(compareMatrices(A, matrixFromCSV, 1e-3));

    gsl_matrix_free(A);
}
