#include "simulator.h"
#include <cwchar>
#include <sys/wait.h>
#include "gantry.h"

// System class
state_t System::ctr_function(const state_t &x, const double t) {
    input_t u(22);
    for(auto &i: u) i = 0.0;
    return u;
}

// Class GantryReduc
GantryReduc::GantryReduc() : A(8,8), b(8), pm(8),state_size(8) {

}

void GantryReduc::operator()(const state_t &x, state_t &dxdt, const double t) {
    input_t u = System::ctr_function(x, t);
    std::copy(x.begin()+6, x.end(), dxdt.begin());
    this->b = this->get_accel(x,dxdt,u);
    std::copy(this->b.begin(), this->b.end(), dxdt.begin()+6);
}

boost::numeric::ublas::vector<double>& GantryReduc::get_accel(const state_t &x, const state_t &dxdt, const input_t &u) {

    init_matrix_A_reduc(this->A, x);
    init_matrix_B_reduc(this->b, x, dxdt, u);

    int singular = boost::numeric::ublas::lu_factorize(A, pm);
    if(singular) {
        std::cout << "Singular matrix!" << std::endl;
    }
    boost::numeric::ublas::lu_substitute(A, pm, b);
    return b;
}


// Class GantryCompl
GantryCompl::GantryCompl() : A(11,11), b(11), pm(11), state_size(11) {

}

void GantryCompl::operator()(const state_t &x, state_t &dxdt, const double t) {
    input_t u = System::ctr_function(x, t);
    std::copy(x.begin()+10, x.end(), dxdt.begin());
    this->b = this->get_accel(x,dxdt,u);
    std::copy(this->b.begin(), this->b.end(), dxdt.begin()+10);
}

boost::numeric::ublas::vector<double>& GantryCompl::get_accel(const state_t &x, const state_t &dxdt, const input_t &u) {

    init_matrix_A_comp(this->A, x);
    init_matrix_B_comp(this->b, x, dxdt, u);

    int singular = boost::numeric::ublas::lu_factorize(A, pm);
    if(singular) {
        std::cout << "Singular matrix!" << std::endl;
    }
    boost::numeric::ublas::lu_substitute(A, pm, b);
    return b;
}

// Simulator class
Simulator::Simulator(System &system): system(system) {

}

state_t Simulator::integrate(state_t &x, const double dt=0.1,const double t0=0.0,const double tf=1.0) {
    boost::numeric::odeint::integrate(this->system, x, t0, tf, dt);
    return x;
}


void free_memory(double *ptr) { delete[] ptr; }
