#include <matplot/matplot.h>

#include <iostream>

#include "ElasticLinearDrive.h"
#include "cell_based_search.h"

void plotTrajectory(const std::vector<std::vector<double>> &traj, const double &time) {
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
  for (auto it = t.begin() + 1; it != t.end(); it++) *it = *(it - 1) + dt;

  plot(t, s);
  show();
}

int main() {
  using namespace matplot;
  std::vector<double> x = {0, 0, 0, 0};
  std::vector<double> xg = {0, 0, 4, 0};

  // auto motion_primtives = ElasticLinearDrive::getMotionPrimitives(x);
  cs::input_trajectory_t input_trajectory = ElasticLinearDrive::getInputTrajector(100000);

  // Set Configuration Space limits
  cs::Options opt(1000, {
                            std::make_pair(-10, 10),
                            std::make_pair(-1000, 1000),
                            std::make_pair(-10, 10),
                            std::make_pair(-1000, 1000),
                        });

  auto G = cellBasedSearch(x, xg, ElasticLinearDrive::dynamics, ElasticLinearDrive::getMotionPrimitives, opt);

  std::cout << "Success: " << G.get_success() << std::endl;
  std::cout << "Number of Vertices: " << G.size_vertices() << std::endl;

  return 0;
}
