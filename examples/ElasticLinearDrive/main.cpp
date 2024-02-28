#include <matplot/freestanding/plot.h>
#include <matplot/matplot.h>
#include <matplotlibcpp.h>

#include <iostream>

#include "ElasticLinearDrive.h"
#include "cell_based_search.h"

void plotTrajectory(const std::vector<std::vector<double>> &traj, const double &time) {
  namespace plt = matplotlibcpp;
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

  plt::figure();
  plt::plot(t, a);
  plt::plot(t,s);
  plt::figure();
  plt::plot(t, v);
  plt::plot(t,j);
  plt::show();
}

void plotGraph(Graph &G,cs::state_t x0,cs::state_t xg) {
  namespace plt = matplotlibcpp;
  std::vector<double> x, y, u, v;

  for ([[maybe_unused]] const auto &[state, out, in] : G) {
    if (state->size() == 4) {
   //   std::cout << "State: " << state->at(0) << " " << state->at(2)
   //             << " Speed: " << state->at(1) << " " << state->at(3) << std::endl;
      x.push_back(state->at(0));  // x position
      y.push_back(state->at(2));  // y position
      u.push_back(state->at(1));  // x component of the arrow
      v.push_back(state->at(3));  // y component of the arrow
    }
  }
  plt::subplot(2, 1, 1);
  plt::plot(x,y,"x");
  plt::plot({x0[0]}, {x0[2]}, "bx");
  plt::plot({xg[0]}, {xg[2]}, "gx");
  plt::subplot(2, 1, 2);
  plt::quiver(x, y, u, v);
  plt::show();
}

void plotVoronoi(cspace::Voronoi &P) {
  namespace plt = matplotlibcpp;
  std::vector<double> x, y, xd, yd;
  for (const auto &p : P) {
    x.push_back(p[0]);
    y.push_back(p[2]);
    xd.push_back(p[1]);
    yd.push_back(p[3]);
  }
  plt::plot(x, y, "o");
  plt::show();
}

int main() {
  using namespace matplot;
  std::vector<double> x = {0, 0, 0, 0};
  std::vector<double> xg = {0, 0, 3, 0};

  // Set Configuration State limits
  cs::Options opt(20000, {
                            std::make_pair(-10, 10),
                            std::make_pair(-20, 20),
                            std::make_pair(-10, 10),
                            std::make_pair(-20, 20),
                        });

  auto [G,P] =
      cellBasedSearch(x, xg, ElasticLinearDrive::dynamics, ElasticLinearDrive::getMotionPrimitives, opt);

  std::cout << "Success: " << G.get_success() << std::endl;
  std::cout << "Number of Vertices: " << G.size_vertices() << std::endl;
  plotGraph(G,x,xg);
  if (G.get_success()) {
    auto state_ptr = G.back().state;
    auto [input_traj,time] = G.get_input(state_ptr);
    auto traj = ElasticLinearDrive::eulerIntegrate(*state_ptr, input_traj, time);
    plotTrajectory(traj, time);
  }

  return 0;
}
