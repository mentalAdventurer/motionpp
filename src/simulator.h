#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "gantry.h"

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_odeiv2.h>

extern "C"{
    double* simulate(double* u, double* x, double* t, int n);
    void free_memory(double* y);
}
int double_integrator(double t, const double y[], double dydt[], void *params);
int gantry_ode(double t, const double y[], double dydt[], void *params);

namespace comp {
    int get_matrix_A(gsl_matrix* A, const double x[]);
}
#endif // SIMULATOR_H
