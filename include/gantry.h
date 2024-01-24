#ifndef GANTRY_H
#define GANTRY_H

#include "simulator.h"
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <sys/wait.h>


void init_matrix_A_reduc(boost::numeric::ublas::matrix<double> &A, const state_t &x);
void init_matrix_B_reduc(boost::numeric::ublas::vector<double> &b, const state_t &x, const state_t &xp, const input_t &u);
int init_matrix_A_comp(boost::numeric::ublas::matrix<double> &A, const state_t &x);
int init_matrix_B_comp(boost::numeric::ublas::vector<double> &b, const state_t &x, const state_t &xp, const input_t &u);


#endif // GANTRY_H
