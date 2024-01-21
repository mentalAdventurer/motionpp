#ifndef GANTRY_H
#define GANTRY_H

#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <math.h>

namespace gantry {
    int get_matrix_A(gsl_matrix *A, const double x[]);
    int get_matrix_B(gsl_vector *B, const double x[], const double xp[], const double u[]);
}

#endif // GANTRY_H
