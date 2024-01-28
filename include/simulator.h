#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <cwchar>
#include <vector>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <cspace.h>

using namespace cspace;

class Gantry{
    public:
        Gantry();
        state_t operator()(const state_t &x, const input_t &);
        std::vector<input_trajectory_t> generate_input_set(int time_steps);

    private:
        //Boost Lib
        boost::numeric::ublas::vector<double> get_accel(const state_t &x, const state_t &dxdt, const input_t &u);
        boost::numeric::ublas::matrix<double> get_accel_matrix_A(const state_t &q);
        boost::numeric::ublas::vector<double> get_accel_vector_B(const state_t &x, const input_t &u);

};

class Simulator {
    public:
        Simulator(Gantry system) : system(system) {};
        state_t explicit_euler(const state_t& x0, const double dt, const input_trajectory_t& u);
    private:
        Gantry system;
};


#endif // SIMULATOR_H
