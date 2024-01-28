#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <sys/wait.h>
#include "cspace.h"
#include "simulator.h"


boost::numeric::ublas::matrix<double> Gantry::get_accel_matrix_A(const cspace::state_t &x) {
    boost::numeric::ublas::matrix<double> A(5,5);
    return A;
}

boost::numeric::ublas::vector<double> Gantry::get_accel_vector_B(const cspace::state_t &x, const cspace::state_t &dxdt, const cspace::input_t &u) {
    boost::numeric::ublas::vector<double> b(5);
    return b;
}
