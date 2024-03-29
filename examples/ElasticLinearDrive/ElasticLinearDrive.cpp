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
  dxdt[1] = (-d * x[1] - k * x[0] + *u) / m;
  dxdt[2] = x[3];
  dxdt[3] = dxdt[1] - *u / m;

  return dxdt;
}

cs::trajectory_t eulerIntegrate_state(const cs::state_t &x0,
                                const cs::input_trajectory_t &u,
                                const float &t) {
  const std::size_t time_steps = u.size();
  const float dt = t / time_steps;

  cs::trajectory_t x_traj(time_steps + 1);
  x_traj[0] = x0;
  cs::state_t x = x0;

  for (std::size_t i = 0; i < time_steps; i++) {
    cs::state_t dxdt = dynamics(x.begin(), u.begin()+i, x.size());
    for (std::size_t j = 0; j < x.size(); j++)
      x[j] = x[j] + dt * dxdt[j];
    x_traj[i + 1] = x;
  }
  return x_traj;
}

std::vector<double> eulerIntegrate(std::vector<double>::const_iterator x0,
                                               std::vector<double>::const_iterator u_traj,
                                               std::size_t state_dim, std::size_t traj_dim, float dt) {
  std::vector<double> x_traj(traj_dim * state_dim + state_dim);
  std::copy(x0, x0 + state_dim, x_traj.begin());

  for (std::size_t j = 0; j < traj_dim; j++) {
    auto dxdt = dynamics(x_traj.begin() + j * state_dim, u_traj + j * state_dim, state_dim);
    for (std::size_t k = 0; k < state_dim; k++) {
      x_traj[j * state_dim + k + state_dim] = x_traj[j * state_dim + k] + dxdt[k] * dt;
    }
  }
  return x_traj;
}

std::pair<std::vector<double>, std::vector<float>> getMotionPrimitives(const cs::state_t &x0) {
  std::vector<float> times(10,0.5); // increase precision and range
  std::size_t dim = 1; // const
  std::size_t step = 6; // increase precision
  double max_input = 4; // increase range

  std::vector<double> motion_primitives(dim * step * times.size());
  double delta_input = 2 * max_input / (times.size() - 1);

  for (std::size_t i = 0; i < times.size(); i++) 
    for (std::size_t j = 0; j < dim*step; j++) 
      motion_primitives[j+i*dim*step] = -max_input + delta_input * i;

  return std::make_pair(motion_primitives, times);
}

cs::input_trajectory_t getInputTrajector(const std::size_t &n) {
  cs::input_trajectory_t input_trajectory(n);
  for (auto &u : input_trajectory)
    u = {0};
  return input_trajectory;
}

} // namespace ElasticLinearDrive
