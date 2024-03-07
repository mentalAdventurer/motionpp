#include <matplot/freestanding/plot.h>
#include <matplot/matplot.h>
#include <iostream>

#include "ElasticLinearDrive.h"
#include "cell_based_search.h"

void plotTrajectory(const std::vector<std::vector<double>> &traj, const double &time) {
  namespace plt = matplot;
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
  plt::plot(t, s);
  plt::title("Trajectory");
  plt::figure();
  plt::plot(t, v);
  plt::plot(t, j);
  plt::show();
}

void plotGraph(Graph &G, cs::state_t x0, cs::state_t xg) {
  namespace plt = matplot;
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
  plt::figure();
  plt::plot(x, y, "x");
  plt::plot({x0[0]}, {x0[2]}, "bx");
  plt::plot({xg[0]}, {xg[2]}, "gx");
  plt::figure();
  plt::quiver(x, y, u, v);
}

void plotVoronoi(cspace::Voronoi &P) {
  namespace plt = matplot;
  std::vector<double> x, y, xd, yd;
  for (const auto &p : P) {
    x.push_back(p[0]);
    y.push_back(p[2]);
    xd.push_back(p[1]);
    yd.push_back(p[3]);
  }
  plt::plot(x, y, "o");
}

void plotStates(const std::vector<cspace::state_t> &states) {
  namespace plt = matplot;
  std::vector<double> x, y, xd, yd;
  for (const auto &s : states) {
    x.push_back(s[2]);
    y.push_back(s[0]);
    xd.push_back(s[3]);
    yd.push_back(s[1]);
  }
  plt::grid(true);
  plt::plot(x);
  plt::plot(y);
  plt::plot(xd);
  plt::plot(yd);
}

int main() {
  namespace plt = matplot;
  std::vector<double> x = {0, 0, 0, 0};
  std::vector<double> xg = {0, 0, 4, 0};

  // Set Configuration State limits
  cs::Options opt(20000, {
                             std::make_pair(-10, 10),
                             std::make_pair(-10, 10),
                             std::make_pair(-10, 10),
                             std::make_pair(-10, 10),
                         });

  auto [G, P] =
      cellBasedSearch(x, xg, opt, ElasticLinearDrive::getMotionPrimitives, ElasticLinearDrive::eulerIntegrate);

  std::cout << "Success: " << G.get_success() << std::endl;
  std::cout << "Number of Vertices: " << G.size_vertices() << std::endl;
  std::cout << "Initial State: " << G.front().state->at(0) << " " << G.front().state->at(1) << " "
            << G.front().state->at(2) << " " << G.front().state->at(3) << std::endl;
  std::cout << "Final State: " << G.back().state->at(0) << " " << G.back().state->at(1) << " "
            << G.back().state->at(2) << " " << G.back().state->at(3) << std::endl;
  bool plot1 = false;
  bool plot2 = false;
  bool plot3 = true;

  if (plot1) plotGraph(G, x, xg);

  if (G.get_success() && plot2) {
    auto state_ptr = G.back().state;
    auto traj = G.get_trajectory(state_ptr);
    plotStates(traj);
  }

  if (G.get_success() && plot3) {
    auto state_ptr = G.back().state;
    auto [input_traj, time] = G.get_input(state_ptr);
    auto traj = ElasticLinearDrive::eulerIntegrate_state(*state_ptr, input_traj, time);
    plotTrajectory(traj, time);
  }
  plt::show();

  return 0;
}
