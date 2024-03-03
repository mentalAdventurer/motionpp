#include "ElasticLinearDrive.h"
#include <iostream>
#include <matplot/matplot.h>

void plotTrajectory(const std::vector<std::vector<double>> &traj,
                    const double &time) {
  using namespace matplot;

  // Calculate Time
  const double steps = traj.size();
  const double dt = time / steps;

  std::vector<double> s;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> j;

  for (auto &x : traj) {
    s.push_back(x[0]);
    v.push_back(x[1]);
    a.push_back(x[2]);
    j.push_back(x[3]);
  }

  // Create time vector size of s
  std::vector<double> t(steps);
  t[0] = 0;
  for (auto it = t.begin() + 1; it != t.end(); it++)
    *it = *(it - 1) + dt;

  plot(t, s);
  show();
}

int main() {
  using namespace matplot;
  std::vector<double> x = {1, 0, 0, 0};
  double time = 30;
  // auto motion_primtives = ElasticLinearDrive::getMotionPrimitives(x);
  cs::input_trajectory_t input_trajectory =
      ElasticLinearDrive::getInputTrajector(100000);

  auto xg = ElasticLinearDrive::eulerIntegrate_state(x, input_trajectory, time);

  // Print the trajectory
  std::cout << "Trajectory: " << xg.size() << std::endl;

  // Plot the trajectory
  plotTrajectory(xg, time);

  return 0;
}
