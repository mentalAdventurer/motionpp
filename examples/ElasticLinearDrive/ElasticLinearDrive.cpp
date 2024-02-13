#include "ElasticLinearDrive.h"
#include "cspace.h"

namespace cs = cspace;
namespace ElasticLinearDrive {

cs::state_t dynamics(const cs::state_t &x, const cs::input_t &u) {
  const size_t n = x.size();
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
    cs::state_t dxdt = dynamics(x, u[i]);
    for (std::size_t j = 0; j < x.size(); j++)
      x[j] = x[j] + dt * dxdt[j];
    x_traj[i + 1] = x;
  }
  return x_traj;
}

std::vector<cs::input_trajectory_t> getMotionPrimitives(const cs::state_t &x0) {
  std::vector<cs::input_trajectory_t> motion_primitives(2);
  motion_primitives[0] = {{1}, {0}};
  motion_primitives[1] = {{1}, {1}};
  return motion_primitives;
}

cs::input_trajectory_t getInputTrajector(const std::size_t &n) {
  cs::input_trajectory_t input_trajectory(n);
  for (auto &u : input_trajectory)
    u = {0};
  return input_trajectory;
}

} // namespace ElasticLinearDrive
