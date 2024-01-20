#ifndef SIMULATOR_H
#define SIMULATOR_H
extern "C"{
    double* simulate(double* u, double* x, double* t, int n);
    void free_memory(double* y);
}
int ode_system(double t, const double y[], double dydt[], void *params);
#endif // SIMULATOR_H
