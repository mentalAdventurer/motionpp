#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <gsl/gsl_matrix.h>


std::vector<std::vector<double>> readMatrixFromCSV(const std::string& filename);
bool compareMatrices(const gsl_matrix* A, const std::vector<std::vector<double>>& B, double tolerance);
