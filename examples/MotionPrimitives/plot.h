#include <matplot/matplot.h>

#include "cell_based_search.h"
#include "cspace.h"
#include "inputShaping.h"
#include "model.h"

using namespace cspace;

void plot_states_and_input(cvec_double& time, cvec_double& states, cvec_double& input, std::size_t dim = 4) {
  namespace plt = matplot;

  // Repackage the states
  std::vector<double> x, xd, z, zd;

  for (std::size_t i = 4; i < states.size(); i += dim) {
    x.push_back(states.at(i));
    xd.push_back(states.at(i + 1));
    z.push_back(states.at(i + 2));
    zd.push_back(states.at(i + 3));
  }
  // Plot the states
  auto h = plt::figure(true);
  h->size(800, 800);
  h->position({0, 0, 600, 600});
  plt::tiledlayout(3, 1);

  plt::nexttile();
  plt::plot(time, x)->line_width(2);
  plt::hold(true);
  plt::plot(time, xd)->line_width(2);
  plt::title("x and x_p");
  plt::grid(true);
  plt::legend({"Position", "Speed"});

  plt::nexttile();
  plt::plot(time, z)->line_width(2);
  plt::hold(true);
  plt::grid(true);
  plt::plot(time, zd)->line_width(2);
  plt::title("z and z_p");

  plt::nexttile();
  plt::plot(time, input)->line_width(2);
  plt::title("Input");
  plt::grid(true);
  plt::xlabel("Time (s)");
  plt::show();
}

void plot_primitives(std::vector<trajTuple>& primitives) {
  namespace plt = matplot;
  auto h = plt::figure(true);
  h->size(800, 800);
  h->position({0, 0, 600, 600});
  plt::tiledlayout(3, 1);
  auto ax0 = plt::nexttile();
  plt::title(ax0, "z");
  plt::hold(ax0, true);
  plt::grid(ax0, true);
  auto ax1 = plt::nexttile();
  plt::title(ax1, "z_p");
  plt::hold(ax1, true);
  plt::grid(ax1, true);
  auto ax2 = plt::nexttile();
  plt::title(ax2, "Input");
  plt::hold(ax2, true);
  plt::grid(ax2, true);
  plt::xlabel(ax2, "Time (s)");

  for (const auto& [time, states, input] : primitives) {
    std::vector<double> x, xd, z, zd;
    for (std::size_t i = 4; i < states.size(); i += 4) {
      x.push_back(states[i]);
      xd.push_back(states[i + 1]);
      z.push_back(states[i + 2]);
      zd.push_back(states[i + 3]);
    }

    plt::plot(ax0, time, z)->line_width(2);
    plt::plot(ax1, time, zd)->line_width(2);
    plt::plot(ax2, time, input)->line_width(2);
  }
  plt::show();
}

void plot_graph(Graph& G, cspace::state_t x0, cspace::state_t xg) {
  namespace plt = matplot;
  std::vector<double> x, xp, z, zp;

  for ([[maybe_unused]] const auto& [state, out, in] : G) {
    if (state->size() == 4) {
      x.push_back(state->at(0));
      xp.push_back(state->at(1));
      z.push_back(state->at(2));
      zp.push_back(state->at(3));
    }
  }
  plt::figure();
  plt::plot(z, zp, ".")->marker_size(5);
  plt::hold(true);
  plt::plot({x0[2]}, {x0[3]}, "bx")->marker_size(15);
  plt::plot({xg[2]}, {xg[3]}, "gx")->marker_size(15);
  plt::legend({"States", "Start", "Goal"});
  plt::grid(true);
  plt::xlabel("z");
  plt::ylabel("z_p");
  plt::show();
}

void plot_trajectory(const std::vector<std::vector<double>>& traj) {
  namespace plt = matplot;
  // Calculate Time

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

  plt::figure();
  plt::plot(z, zp)->line_width(2);
  plt::hold(true);
  plt::plot(z, zp, ".")->marker_size(5);
  plt::xlabel("z");
  plt::ylabel("z_p");
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
