#include "gtest/gtest.h"
#include "utils_test.h"
#define PATH_TO_MATRIX_0 "../../tests/Matrix/test_matrix_comp_0.csv"
#define PATH_TO_MATRIX_1 "../../tests/Matrix/test_matrix_comp_1.csv"
#define PATH_TO_MATRIX_3 "../../tests/Matrix/test_matrix_reduc_0.csv"
#define PATH_TO_MATRIX_4 "../../tests/Matrix/test_matrix_reduc_1.csv"

// Reduced Model
TEST(ModelTestGantryReduc, CompareMatrixSizes) {
    const size_t size = 8;
    state_t x(8,0.0); // Adjust as needed for your test
    boost::numeric::ublas::matrix<double> A(size, size); 
    auto B = readMatrixFromCSV(PATH_TO_MATRIX_3); // Read matrix from CSV"); 

    init_matrix_A_reduc(A, x); // Populate A using your function 
    size_t rows_A = A.size1(); 
    size_t cols_A = A.size2(); 
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

TEST(ModelTestGantryReduc, ZeroInputCheck) {
    const size_t size = 8;
    state_t x(size,0.0); // Adjust as needed for your test
    boost::numeric::ublas::matrix<double> A(size, size); 

    init_matrix_A_reduc(A, x); // Populate A using your function 

    auto matrixFromCSV = readMatrixFromCSV(PATH_TO_MATRIX_3); // Read matrix from CSV

    EXPECT_TRUE(compareMatrices(A, matrixFromCSV, 1e-3)); // Compare matrices
}

TEST(ModelTestGantryReduc, PseudoRandomInputCheck) {
    const size_t size = 8;
    state_t x =  {30.0,105.142142,31.12312,23,23,0.234234,0.24234,100};
    boost::numeric::ublas::matrix<double> A(size,size);

    init_matrix_A_reduc(A, x); // Populate A using your function 

    auto matrixFromCSV = readMatrixFromCSV(PATH_TO_MATRIX_4);

    EXPECT_TRUE(compareMatrices(A, matrixFromCSV, 1e-3));

}

// Complete Model
TEST(ModelTestGantryComp, CompareMatrixSizes) {
    const size_t size = 11;
    state_t x(11,0.0); // Adjust as needed for your test
    boost::numeric::ublas::matrix<double> A(size, size); 
    auto B = readMatrixFromCSV(PATH_TO_MATRIX_0); // Read matrix from CSV"); 

    init_matrix_A_comp(A, x); // Populate A using your function 
    size_t rows_A = A.size1(); 
    size_t cols_A = A.size2(); 
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

TEST(ModelTestGantryComp, ZeroInputCheck) {
    const size_t size = 11;
    state_t x(11,0.0); // Adjust as needed for your test
    boost::numeric::ublas::matrix<double> A(size, size); 
    auto B = readMatrixFromCSV(PATH_TO_MATRIX_0); // Read matrix from CSV"); 

    init_matrix_A_comp(A, x); // Populate A using your function 

    auto matrixFromCSV = readMatrixFromCSV(PATH_TO_MATRIX_0); // Read matrix from CSV

    EXPECT_TRUE(compareMatrices(A, matrixFromCSV, 1e-3)); // Compare matrices
}

TEST(ModelTestGantryComp, PseudoRandomInputCheck) {
    const size_t size = 11;
    state_t x =  {30.0,1005.142142,31.12312,23,23,0.234234,0.24234,100,1,5,0};
    boost::numeric::ublas::matrix<double> A(size,size);

    init_matrix_A_comp(A, x); // Populate A using your function 

    auto matrixFromCSV = readMatrixFromCSV(PATH_TO_MATRIX_1);

    EXPECT_TRUE(compareMatrices(A, matrixFromCSV, 1e-3));

}
