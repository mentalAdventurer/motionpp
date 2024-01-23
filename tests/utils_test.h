#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>


std::vector<std::vector<double>> readMatrixFromCSV(const std::string& filename);
bool compareMatrices(const boost::numeric::ublas::matrix<double>& A, const std::vector<std::vector<double>>& B, double tolerance);
