#include "ElasticLinearDrive.h"
#include "cspace.h"
#include <cassert>

namespace cs = cspace;
namespace ElasticLinearDrive {

// create alias for std::vector<double>::const_iterator
using vec_iter = std::vector<double>::const_iterator;

std::vector<double> dynamics(vec_iter x, vec_iter u, std::size_t n) {
  assert (n == 4); 
  const float d = 0.1;
  const float k = 1;
  const float m = 1;

  cs::state_t dxdt(n);
  dxdt[0] = x[1];
  dxdt[1] = (-d * x[1] - k * x[0] + u[0]) / m;
  dxdt[2] = x[3];
  dxdt[3] = dxdt[1] - u[0] / m;

  return dxdt;
}

cs::trajectory_t eulerIntegrate(const cs::state_t &x0,
                                const cs::input_trajectory_t &u,
                                const float &t) {
  const std::size_t time_steps = u.size();
  const float dt = t / time_steps;

  cs::trajectory_t x_traj(time_steps + 1);
  x_traj[0] = x0;
  cs::state_t x = x0;

  for (std::size_t i = 0; i < time_steps; i++) {
    cs::state_t dxdt = dynamics(x.begin(), u[i].begin(), x.size());
    for (std::size_t j = 0; j < x.size(); j++)
      x[j] = x[j] + dt * dxdt[j];
    x_traj[i + 1] = x;
  }
  return x_traj;
}

std::pair<std::vector<double>, std::vector<float>> getMotionPrimitives(const cs::state_t &x0) {
  std::vector<double> motion_primitives = {1.0, 2.0, 4.0, 2.7, 1.0, 2.0, 4.0, 2.7};
  std::vector<float> times = {0.1, 0.2};
  return std::make_pair(motion_primitives, times);
}

cs::input_trajectory_t getInputTrajector(const std::size_t &n) {
  cs::input_trajectory_t input_trajectory(n);
  for (auto &u : input_trajectory)
    u = {0};
  return input_trajectory;
}

} // namespace ElasticLinearDrive
