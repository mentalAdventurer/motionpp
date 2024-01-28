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

    private:
        //Boost Lib
        boost::numeric::ublas::vector<double> get_accel(const state_t &x, const state_t &dxdt, const input_t &u);
        boost::numeric::ublas::matrix<double> get_accel_matrix_A(const state_t &x);
        boost::numeric::ublas::vector<double> get_accel_vector_B(const state_t &x, const state_t &dxdt, const input_t &u);

};

class System{
    // This should be an abstract class but can't because of boost::odeint
    // TODO: find a way to make this an abstract class
    public:
        virtual void operator()(const state_t &x, state_t &dxdt, const double t)=0;
        virtual state_t operator()(const state_t &x, const input_t &u)=0;
        virtual ~System(){};
    protected:
        // Acceleartion Equation
        input_t ctr_function(const state_t &x, const double t);
};

class GantryReduc : public System{
    public:
        size_t state_size;
        GantryReduc();
        void operator()(const state_t &x, state_t &dxdt, const double t) override;
        state_t operator()(const state_t &x,const input_t &u) override;

        // Accerlation linear Equation
        friend void init_matrix_A_reduc(boost::numeric::ublas::matrix<double> &A, const state_t &x); 
        friend void init_matrix_B_reduc(boost::numeric::ublas::vector<double> &b, const state_t &x, const state_t &xp, const input_t &u);

    private:
        boost::numeric::ublas::matrix<double> A;
        boost::numeric::ublas::vector<double> b;
        boost::numeric::ublas::permutation_matrix<std::size_t> pm;
        boost::numeric::ublas::vector<double>& get_accel(const state_t &x, const state_t &dxdt, const input_t &u);
};

class GantryCompl : public System{
    public:
        size_t state_size;
        GantryCompl();
        void operator()(const state_t &x, state_t &dxdt, const double t) override;

        // Accerlation linear Equation
        friend int init_matrix_A_comp(boost::numeric::ublas::matrix<double> &A, const state_t &x); 
        friend int init_matrix_B_comp(boost::numeric::ublas::vector<double> &b, const state_t &x, const state_t &xp, const input_t &u);
    private:
        boost::numeric::ublas::matrix<double> A;
        boost::numeric::ublas::vector<double> b;
        boost::numeric::ublas::vector<double>& get_accel(const state_t &x, const state_t &dxdt, const input_t &u);
        boost::numeric::ublas::permutation_matrix<std::size_t> pm;
        
};

class Simulator {
    public:
        Simulator(Gantry system) : system(system) {};
        state_t explicit_euler(state_t &x, const double dt, const input_trajectory_t &u);
    private:
        Gantry system;
};


#endif // SIMULATOR_H
