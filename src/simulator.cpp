#include "simulator.h"
#include <gsl/gsl_odeiv2.h>
#include <gsl/gsl_errno.h>
#include <vector> 


// Create a function called "simulate"
// This function takes three c arrays of doubles as arguments 
// The arrays are u (input), x (initial state), t (time) 
// for now the function returns just an array of the same size with just zeros

// ODE system function
int ode_system(double t, const double y[], double dydt[], void *params) {
    double u = *(double *)params;
    dydt[0] = y[1];
    dydt[1] = u;
    return GSL_SUCCESS;
}

// Simulate function
double* simulate(double* u, double* x, double* t, int n) {
    gsl_odeiv2_system sys = {ode_system, nullptr, 2, u};

    const gsl_odeiv2_step_type *T = gsl_odeiv2_step_rk8pd;
    gsl_odeiv2_step *s = gsl_odeiv2_step_alloc(T, 2);
    gsl_odeiv2_control *c = gsl_odeiv2_control_y_new(1e-6, 0.0);
    gsl_odeiv2_evolve *e = gsl_odeiv2_evolve_alloc(2);

    double y[2] = { x[0], x[1] }; // Initial conditions
    double t0 = t[0];
    double h = 1e-6; // Initial step size

    double* results = new double[n * 2];
    for (int i = 0; i < n; ++i) {
        while (t0 < t[i]) {
            int status = gsl_odeiv2_evolve_apply(e, c, s, &sys, &t0, t[i], &h, y);
            if (status != GSL_SUCCESS) {
                // Handle the error, for example, break the loop
                break;
            }
        }
        results[i*2] = y[0];
        results[i*2 + 1] = y[1];
    }

    gsl_odeiv2_evolve_free(e);
    gsl_odeiv2_control_free(c);
    gsl_odeiv2_step_free(s);

    return results;
}

void free_memory(double* ptr) {
    delete[] ptr;
}
