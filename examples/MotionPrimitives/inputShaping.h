#ifndef INPUTSHAPING_H
#define INPUTSHAPING_H

#include <cmath>
#include <tuple>
#include <vector>

using scurve = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>>;
using traj = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;
using vec_double = std::vector<double>;
using cvec_double = const std::vector<double>;

scurve generate_scurve(const double start_point, const double end_point, double speed_limit,
                       double acceleration_limit, const double dt);

vec_double generate_impulse_from_signal(cvec_double &signal);
traj generate_traj_from_impulses(cvec_double &impulse, cvec_double &time);
std::pair<vec_double, vec_double> apply_zero_vibration_shaping(cvec_double &impulse, vec_double time,
                                                               double kappa, double Tv);

#endif  // INPUTSHAPING_H
