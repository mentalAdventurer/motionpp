#include "simulator.h"
#include <cwchar>
#include <sys/wait.h>

// Simple Reduced Class implementation
Gantry::Gantry(){}

state_t Gantry::operator()(const state_t &x, const input_t &u) {
    size_t state_dim = x.size();
    state_t dxdy(state_dim);

    // Velocity
    std::copy(x.begin()+state_dim/2, x.end(), dxdy.begin());
    // Acceleration
    auto qdot = get_accel(x,dxdy,u);
    std::copy(qdot.begin(), qdot.end(), dxdy.begin()+state_dim/2);

    return dxdy;
}

boost::numeric::ublas::vector<double> Gantry::get_accel(const state_t &x, const state_t &dxdt, const input_t &u) {
    auto A = get_accel_matrix_A(x);
    auto b = get_accel_vector_B(x,dxdt,u);
   
    boost::numeric::ublas::permutation_matrix<std::size_t> pm(A.size1());

    int singular = boost::numeric::ublas::lu_factorize(A, pm);
    if(singular) {
        std::cout << "Singular matrix!" << std::endl;
    }
    boost::numeric::ublas::lu_substitute(A, pm, b);

    return b;
}



// Simulator class
state_t Simulator::explicit_euler(state_t &x, const double dt, const input_trajectory_t &u) {

    // Explicit Euler
    state_t dxdt = system(x,u[0]);
    state_t xk(x.size());
    for(int i = 0;i<xk.size();i++) xk[i] = x[i] + dt*dxdt[i]; 
    
    return xk;
}


void free_memory(double *ptr) { delete[] ptr; }
