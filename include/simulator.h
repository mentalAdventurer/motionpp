#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include <boost/numeric/odeint.hpp>

typedef std::vector<double> state_t;
typedef std::vector<double> input_t;
typedef std::vector<double> trajectory_t;

class Gantry{
    public:
        Gantry();
        void operator()(const state_t &x, state_t &dxdt, const double t);

        // Accerlation linear Equation
        friend int init_matrix_A(boost::numeric::ublas::matrix<double> &A, const state_t &x); 
        friend int init_matrix_B(boost::numeric::ublas::vector<double> &b, const state_t &x, const state_t &xp, const input_t &u);
    private:
        // Acceleartion Equation
        boost::numeric::ublas::matrix<double> A;
        boost::numeric::ublas::vector<double> b;
        boost::numeric::ublas::permutation_matrix<std::size_t> pm;
        boost::numeric::ublas::vector<double>& get_accel(const state_t &x, const state_t &dxdt, const input_t &u);
        
        input_t ctr_function(const state_t &x, const double t);

};

class Simulator {
    public:
        Simulator(Gantry &gantry);
        state_t integrate(state_t &x, const double dt,const double t0,const double tf);
    private:
        Gantry &system;
};


#endif // SIMULATOR_H
