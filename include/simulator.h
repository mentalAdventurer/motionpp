#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include <boost/numeric/odeint.hpp>

typedef std::vector<double> state_t;
typedef std::vector<double> input_t;
typedef std::vector<double> trajectory_t;

class System{
    // This should be an abstract class but can't because of boost::odeint
    // TODO: find a way to make this an abstract class
    public:
        virtual void operator()(const state_t &x, state_t &dxdt, const double t){};
        virtual ~System(){};
};

class GantryCompl : public System{
    public:
        GantryCompl();
        void operator()(const state_t &x, state_t &dxdt, const double t) override;

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
        Simulator(System &gantry);
        state_t integrate(state_t &x, const double dt,const double t0,const double tf);
    private:
        System &system;
};


#endif // SIMULATOR_H
