#include <iostream>
#include <matplot/matplot.h>

#include "inputShaping.h"
#include "model.h"


void plot_states_and_input(const cvec_double& time, const cvec_double& states, const cvec_double& input) {
  namespace plt = matplot;
  
  // Repackage the states
  std::size_t dim = 4;
  std::vector<double> x, xd, z, zd;

  for (std::size_t i = 4; i < states.size(); i += dim) {
    x.push_back(states[i]);
    xd.push_back(states[i + 1]);
    z.push_back(states[i + 2]);
    zd.push_back(states[i + 3]);
  }
  // Plot the states
  plt::subplot(4, 1, 1);
  plt::plot(time, x);
  plt::plot(time, xd);
  plt::title("States");
  plt::xlabel("Time (s)");
  plt::ylabel("Position (m) / Speed (m/s)");
  plt::grid(true);

  plt::subplot(4, 1, 2);
  plt::plot(time,z);
  plt::plot(time,zd);
  plt::xlabel("Time (s)");
  plt::ylabel("Position (m) / Speed (m/s)");
  plt::grid(true);

  //plt::subplot(3, 1, 3);
  //plt::plot(time,input);
  //plt::title("Input");
  //plt::xlabel("Time (s)");
  //plt::ylabel("Force (N)");
  //plt::grid(true);
  plt::show();
}

int main() {
  // Define Parameter
  // Define Initial Condition

  double start_position = 0.0, start_speed = 0.0, target_speed = 0.3;

  auto [times, speed, accel, jerk] =
      generate_scurve(start_speed, target_speed, param::accel_limit, param::jerk_limit, param::dt);
  auto impulses = generate_impulse_from_signal(jerk);
  auto [impulse, time] = apply_zero_vibration_shaping(impulses, times, param::kappa, param::Tv);
  auto [zv_v, zv_a, zv_j] = generate_traj_from_impulses(impulse, time);
  auto inputs = calculate_input({0, 0, start_position, start_speed}, zv_a, param::dt);
  auto traj = simulate_system({0, 0, start_position, start_speed}, inputs, param::dt);

  // Morgen: Hier Trajektorie plotten

  // Plot the S-Curve
  plot_states_and_input(time, traj, inputs);
}
