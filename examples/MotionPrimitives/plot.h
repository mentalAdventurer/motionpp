#include <matplot/matplot.h>
#include "cspace.h"
#include "cell_based_search.h"
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
  plt::grid(ax0,true);
  auto ax1 = plt::nexttile();
  plt::title(ax1, "z_p");
  plt::hold(ax1, true);
  plt::grid(ax1,true);
  auto ax2 = plt::nexttile();
  plt::title(ax2, "Input");
  plt::hold(ax2, true);
  plt::grid(ax2,true);
  plt::xlabel(ax2, "Time (s)");

  for (const auto& [time, states, input] : primitives) {
    std::vector<double> x, xd, z, zd;
    for (std::size_t i = 4; i < states.size(); i += 4) {
      x.push_back(states[i]);
      xd.push_back(states[i + 1]);
      z.push_back(states[i + 2]);
      zd.push_back(states[i + 3]);
    }

    plt::plot(ax0,time, z)->line_width(2);
    plt::plot(ax1,time, zd)->line_width(2);
    plt::plot(ax2,time, input)->line_width(2);
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

void plot_trajectory(const std::vector<std::vector<double>> &traj) {
  namespace plt = matplot;
  // Calculate Time

  std::vector<double> x;
  std::vector<double> xp;
  std::vector<double> z;
  std::vector<double> zp;

  for (auto &xi : traj) {
    x.push_back(xi[0]);
    xp.push_back(xi[1]);
    z.push_back(xi[2]);
    zp.push_back(xi[3]);
  }

  plt::figure();
  plt::plot(z, zp)->line_width(2);
  plt::hold(true);
  plt::plot(z, zp,".")->marker_size(5);
  plt::xlabel("z");
  plt::ylabel("z_p");
  plt::title("Trajectory");
  plt::show();
}
