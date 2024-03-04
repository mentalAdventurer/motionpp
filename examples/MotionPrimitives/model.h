#ifndef MODEL_H
#define MODEL_H
#include "inputShaping.h"
#include <cmath>

namespace param {
static constexpr double m2 = 4.6546, m1 = 26.9057, k = 117498.8867, d = 50.4189; 
static constexpr double accel_limit = 6, jerk_limit = 200, dt = 1e-4;

static constexpr double mg = m1+m2, delta = d/ (2*mg), w0_sq = k/mg; 
static const double wd = std::sqrt(w0_sq - delta*delta), fd = wd / (2 * M_PI), Tv = 1 / (2 * fd);
static const double kappa = std::exp(-Tv * delta); 
}  // namespace param
   
   

vec_double dynamics(cvec_double& x,double u);
vec_double calculate_input(vec_double x0, cvec_double accel, double dt);
vec_double simulate_system(vec_double x, cvec_double& inputs, double dt);
#endif  // MODEL_H
