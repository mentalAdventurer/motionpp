#include <matplot/matplot.h>

#include "cell_based_search.h"
#include "cspace.h"
#include "inputShaping.h"
#include "model.h"

using namespace cspace;

void plot_states_and_input(cvec_double& time, cvec_double& states, cvec_double& input, std::size_t dim = 8) {
  namespace plt = matplot;

  // Repackage the states
  std::vector<double> x, xd, z, zd, x_x, xd_x, z_x, zd_x, x_y, xd_y, z_y, zd_y;

  for (std::size_t i = 8; i < states.size(); i += dim) {
    x_x.push_back(states.at(i));
    xd_x.push_back(states.at(i + 1));
    z_x.push_back(states.at(i + 2));
    zd_x.push_back(states.at(i + 3));
    x_y.push_back(states.at(i + 4));
    xd_y.push_back(states.at(i + 5));
    z_y.push_back(states.at(i + 6));
    zd_y.push_back(states.at(i + 7));
  }
  // Plot the states
  auto h = plt::figure(true);
  h->size(800, 800);
  h->position({0, 0, 600, 600});

  plt::plot(time, x_y)->line_width(2);
  plt::title("Position");
  plt::grid(true);
  plt::show();
}

void plot_primitives(std::vector<trajTuple>& primitives) {
  namespace plt = matplot;
  auto h = plt::figure(true);
  h->size(1600, 800);
  h->position({0, 0, 600, 600});
  plt::tiledlayout(1, 2);
  auto ax0 = plt::nexttile();
  plt::title(ax0, "Position Placer");
  plt::hold(ax0, true);
  plt::grid(ax0, true);
  auto ax1 = plt::nexttile();
  plt::title(ax1, "Position Base");
  plt::hold(ax1, true);
  plt::grid(ax1, true);

  for (const auto& [time, states, input] : primitives) {
    std::vector<double> x, xd, z, zd, x_x, xd_x, z_x, zd_x, x_y, xd_y, z_y, zd_y;
    for (std::size_t i = 8; i < states.size(); i += 8) {
      x_x.push_back(states.at(i));
      xd_x.push_back(states.at(i + 1));
      z_x.push_back(states.at(i + 2));
      zd_x.push_back(states.at(i + 3));
      x_y.push_back(states.at(i + 4));
      xd_y.push_back(states.at(i + 5));
      z_y.push_back(states.at(i + 6));
      zd_y.push_back(states.at(i + 7));
    }

    plt::plot(ax0, z_x, z_y)->line_width(2);
    plt::plot(ax1, x_x, x_y)->line_width(2);
  }
  plt::show();
}

void plot_graph(Graph& G, cspace::state_t x0, cspace::state_t xg,
                const std::vector<std::vector<double>>& traj = {},
                const std::vector<cspace::StaticObstacle>& obstacles = {}) {
  namespace plt = matplot;
  std::vector<std::vector<double>> graph_repacked(8);

  for (const auto& [states, out, in] : G) {
    if (states->size() == 8) {
      for (std::size_t i = 0; i < 8; i++) {
        graph_repacked[i].push_back(states->at(i));
      }
    }
  }
  plt::figure();
  plt::plot(graph_repacked[2], graph_repacked[6], ".")->marker_size(5);
  plt::hold(true);
  plt::plot({x0[2]}, {x0[6]}, "bx")->marker_size(15);
  plt::plot({xg[2]}, {xg[6]}, "gx")->marker_size(15);
  plt::grid(true);
  plt::xlabel("z_x");
  plt::ylabel("z_y");

  // Plot the trajectory
  if (!traj.empty()) {
    std::vector<std::vector<double>> traj_repacked(8);
    for (auto& xi : traj) {
      for (std::size_t i = 0; i < 8; i++) {
        traj_repacked[i].push_back(xi[i]);
      }
    }
    plt::plot(traj_repacked[2], traj_repacked[6])->line_width(2);
    plt::plot(traj_repacked[2], traj_repacked[6], ".")->marker_size(5);
  }
  // Plot the obstacles
  for (auto obj : obstacles) {
    std::vector<double> x, y;
    for (auto& vertex : obj.polytope_data) {
      x.push_back(vertex[0]);
      y.push_back(vertex[1]);
    }
    // Close the Obstacle
    x.push_back(obj.polytope_data[0][0]);
    y.push_back(obj.polytope_data[0][1]);
    plt::plot(x, y, "r-")->line_width(2);
  }

  plt::show();
}

void plot_trajectory(const std::vector<std::vector<double>>& traj) {
  namespace plt = matplot;
  // Calculate Time

  std::vector<double> x, xd, z, zd, x_x, xd_x, z_x, zd_x, x_y, xd_y, z_y, zd_y;

  for (auto& xi : traj) {
    x_x.push_back(xi[0]);
    xd_x.push_back(xi[1]);
    z_x.push_back(xi[2]);
    zd_x.push_back(xi[3]);
    x_y.push_back(xi[4]);
    xd_y.push_back(xi[5]);
    z_y.push_back(xi[6]);
    zd_y.push_back(xi[7]);
  }

  plt::figure();
  plt::plot(z_x, z_y)->line_width(2);
  plt::hold(true);
  plt::plot(z_x, z_y, ".")->marker_size(5);
  plt::xlabel("z_x");
  plt::ylabel("z_y");
  plt::title("Trajectory");
  plt::show();
}

void plot_trajectory_time(const std::vector<std::vector<double>>& traj, const std::vector<double>& time) {
  namespace plt = matplot;
  // Calculate Time
  const double steps = traj.size();
  const double dt = time.back() / steps;

  std::vector<double> x;
  std::vector<double> xp;
  std::vector<double> z;
  std::vector<double> zp;

  for (auto& xi : traj) {
    x.push_back(xi[0]);
    xp.push_back(xi[1]);
    z.push_back(xi[2]);
    zp.push_back(xi[3]);
  }

  // Create time vector size of s
  std::vector<double> t(steps);
  t[0] = 0;
  for (auto it = t.begin() + 1; it != t.end(); it++) *it = *(it - 1) + dt;

  plt::figure();
  plt::plot(t, zp)->line_width(2);
  plt::hold(true);
  plt::plot(t, zp, ".")->marker_size(5);
  plt::title("Trajectory");
  plt::xlabel("Time (s)");
  plt::ylabel("z_p");
  plt::show();
}

void plot_trajectory_time(cspace::trajectory_t traj, double time) {
  namespace plt = matplot;

  const double steps = traj.size();
  const double dt = time / steps;
  std::vector<double> t(steps);
  // initialize t
  double t_curr = 0;
  for (auto it = t.begin(); it != t.end(); it++) {
    *it = t_curr;
    t_curr += dt;
  }

  std::vector<double> x, xp, z, zp;
  for (auto& xi : traj) {
    x.push_back(xi[0]);
    xp.push_back(xi[1]);
    z.push_back(xi[2]);
    zp.push_back(xi[3]);
  }
  // Plot the states
  auto h = plt::figure(true);
  h->size(800, 800);
  h->position({0, 0, 600, 600});
  plt::tiledlayout(2, 1);

  plt::nexttile();
  plt::plot(t, x)->line_width(2);
  plt::hold(true);
  plt::plot(t, xp)->line_width(2);
  plt::title("x and x_p");
  plt::grid(true);
  plt::legend({"Position", "Speed"});

  plt::nexttile();
  plt::plot(t, z)->line_width(2);
  plt::hold(true);
  plt::grid(true);
  plt::plot(t, zp)->line_width(2);
  plt::title("z and z_p");

  plt::show();
}

void plot_trajectory_flat(const std::vector<double>& traj, double time_final) {
  namespace plt = matplot;
  std::vector<double> x, xp, z, zp;
  // Repackage the states
  for (std::size_t i = 0; i < traj.size(); i += 4) {
    x.push_back(traj[i]);
    xp.push_back(traj[i + 1]);
    z.push_back(traj[i + 2]);
    zp.push_back(traj[i + 3]);
  }

  // Gernate time vector
  const double steps = xp.size();
  const double dt = time_final / steps;
  std::vector<double> t(steps);
  double t_curr = 0;
  for (auto it = t.begin(); it != t.end(); it++) {
    *it = t_curr;
    t_curr += dt;
  }

  plt::tiledlayout(2, 1);
  plt::nexttile();
  plt::plot(t, x)->line_width(2);
  plt::hold(true);
  plt::plot(t, xp)->line_width(2);
  plt::legend({"x", "xp"});
  plt::title("Simulated Trajectory lower cart");

  plt::nexttile();
  plt::plot(t, z)->line_width(2);
  plt::hold(true);
  plt::plot(t, zp)->line_width(2);
  plt::xlabel("time");
  plt::legend({"z", "zp"});
  plt::title("Simulated Trajectory upper cart");
  plt::show();
}

void plot_input(cspace::input_trajectory_t input, double time_final) {
  namespace plt = matplot;
  const double steps = input.size();
  const double dt = time_final / steps;
  std::vector<double> t(steps);
  // initialize t
  double t_curr = 0;
  for (auto it = t.begin(); it != t.end(); it++) {
    *it = t_curr;
    t_curr += dt;
  }

  plt::figure();
  plt::plot(t, input)->line_width(2);
  plt::title("Input");
  plt::grid(true);
  plt::xlabel("Time (s)");
  plt::show();
}
