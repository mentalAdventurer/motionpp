#include <matplot/matplot.h>

#include "cell_based_search.h"
#include "cspace.h"

using namespace cspace;

void plot_states(std::vector<double>& time, std::vector<double>& states) {
  namespace plt = matplot;

  // Repackage the states
  std::vector<double> z_x, zd_x, zdd_x, z_y, zd_y, zdd_y; 

  for (std::size_t i = 0; i < states.size(); i += 6) {
    z_x.push_back(states.at(i));
    zd_x.push_back(states.at(i + 1));
    zdd_x.push_back(states.at(i + 2));
    z_y.push_back(states.at(i + 3));
    zd_y.push_back(states.at(i + 4));
    zdd_y.push_back(states.at(i + 5));
  }
  // Plot the states
  auto h = plt::figure(true);
  h->size(800, 800);
  h->position({0, 0, 600, 600});
  plt::tiledlayout(3, 1);

  plt::nexttile();
  plt::title("Position");
  plt::plot(time, z_x)->line_width(2);
  plt::hold(true);
  plt::plot(time, z_y)->line_width(2);
  plt::grid(true);

  plt::nexttile();
  plt::title("Speed");
  plt::plot(time, zd_x)->line_width(2);
  plt::hold(true);
  plt::plot(time, zd_y)->line_width(2);
  plt::grid(true);

  plt::nexttile();
  plt::title("Acceleration");
  plt::plot(time, zdd_x)->line_width(2);
  plt::hold(true);
  plt::plot(time, zdd_y)->line_width(2);
  plt::grid(true);
  plt::show();
}

void plot_primitives(std::vector<trajTuple>& primitives) {
  namespace plt = matplot;
  auto h = plt::figure(true);
  h->size(1600, 800);
  h->position({0, 0, 600, 600});

  plt::tiledlayout(3, 1);
  auto ax0 = plt::nexttile();
  plt::hold(ax0, true);
  plt::grid(ax0, true);
  plt::ylabel(ax0, R"($z\\_x$ (m))");

  auto ax1 = plt::nexttile();
  plt::hold(ax1, true);
  plt::grid(ax1, true);
  plt::ylabel(ax1, R"($\\\\dot\\{z\\}\\_x$ (m/s))");


  auto ax2 = plt::nexttile();
  plt::hold(ax2, true);
  plt::grid(ax2, true);
  plt::xlabel(ax2, "Time (s)");
  plt::ylabel(ax2, R"($\\\\ddot\\{z\\}\\_x$ (m/s$\\^$2))");
  ax2->y_axis();

  for (const auto& [time, states, input] : primitives) {
    std::vector<double> z_x,zd_x,zdd_x;
    for (std::size_t i = 0; i < states.size(); i += 3) {
     z_x.push_back(states.at(i));
     zd_x.push_back(states.at(i+1));
     zdd_x.push_back(states.at(i+2));
    }

    plt::plot(ax0, time, z_x)->line_width(2);
    plt::plot(ax1, time, zd_x)->line_width(2);
    plt::plot(ax2, time, zdd_x)->line_width(2);
  }
  plt::show();
}

void plot_input(cspace::input_trajectory_t input, double time_final) {
  namespace plt = matplot;
  const double steps = input.size()/2;
  const double dt = time_final / steps;
  std::vector<double> t(steps);
  // initialize t
  double t_curr = 0;
  for (auto it = t.begin(); it != t.end(); it++) {
    *it = t_curr;
    t_curr += dt;
  }
  // Repackage the input
  std::vector<double> input_x, input_y;
  for (std::size_t i = 0; i < input.size(); i += 2) {
    input_x.push_back(input[i]);
    input_y.push_back(input[i + 1]);
  }

  plt::figure();
  plt::plot(t, input_x)->line_width(2);
  plt::hold(true);
  plt::plot(t, input_y)->line_width(2);
  plt::title("Input");
  plt::grid(true);
  plt::xlabel("Time (s)");
  plt::show();
}

void plot_graph(Graph& G, cspace::state_t x0, cspace::state_t xg,
                const std::vector<std::vector<double>>& traj = {},
                const std::vector<cspace::StaticObstacle>& obstacles = {}) {
  namespace plt = matplot;
  std::vector<std::vector<double>> graph_repacked(3);

  for (const auto& [states, out, in] : G) {
    if (states->size() == 3) {
      for (std::size_t i = 0; i < 3; i++) {
        graph_repacked[i].push_back(states->at(i));
      }
    }
  }
  plt::figure();
  plt::plot(graph_repacked[0], graph_repacked[1], ".")->marker_size(5);
  plt::hold(true);
  plt::plot({x0[0]}, {x0[1]}, "bx")->marker_size(15);
  plt::plot({xg[0]}, {xg[1]}, "gx")->marker_size(15);
  plt::grid(true);
  plt::xlabel("z_x");
  plt::ylabel("z_y");

  // Plot the trajectory
  if (!traj.empty()) {
    std::vector<std::vector<double>> traj_repacked(6);
    for (auto& xi : traj) {
      for (std::size_t i = 0; i < 6; i++) {
        traj_repacked[i].push_back(xi[i]);
      }
    }
    plt::plot(traj_repacked[0], traj_repacked[1])->line_width(2);
    plt::plot(traj_repacked[0], traj_repacked[1], ".")->marker_size(5);
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
