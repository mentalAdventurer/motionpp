#pragma once

#include <cmath>
#include <vector> 
#include "cspace.h"
#include <span>

namespace param {
static constexpr double m2 = 4.6546, m1 = 26.9057, k = 117498.8867, d = 50.4189; 
static constexpr double accel_limit = 6, jerk_limit = 200, dt = 1e-4;

static constexpr double mg = m1+m2, delta = d/ (2*mg), w0_sq = k/mg; 
const double wd = std::sqrt(w0_sq - delta*delta), fd = wd / (2 * M_PI), Tv = 1 / (2 * fd);
const double kappa = std::exp(-Tv * delta); 
static constexpr double j_max = 333.3333;
static constexpr double a_max = 10;
static constexpr double v_max = 1.5;
static constexpr double s_max = 0.5;
}  // namespace param

using primitiveTuple = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;
using ZVStateInputTimeTuple = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;

struct MotionPrimitives {
  double time_step = 1e-3;
  std::vector<double> accel = {0.0, 10};
  std::vector<std::vector<double>> jerk_impuls;
  std::vector<std::vector<double>> time_impuls;
  MotionPrimitives();
  std::vector<primitiveTuple> operator()(const cspace::state_t x);
  ZVStateInputTimeTuple calculate_zv_from_impuls(std::span<const double> x0, std::vector<double>& jerk_impuls,
                                                 std::vector<double>& time_impuls, double dt,
                                                 double time_end);
};
