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
  plt::tiledlayout(1, 1);
  auto ax0 = plt::nexttile();
  plt::title(ax0, "Position Placer");
  plt::hold(ax0, true);
  plt::grid(ax0, true);

  for (const auto& [time, states, input] : primitives) {
    std::vector<double> z_x,zd_x,zdd_x,z_y,zd_y,zdd_y; 
    for (std::size_t i = 0; i < states.size(); i += 6) {
     z_x.push_back(states.at(i));
     zd_x.push_back(states.at(i+1));
     zdd_x.push_back(states.at(i+2));
     z_y.push_back(states.at(i+3));
     zd_y.push_back(states.at(i+4));
     zdd_y.push_back(states.at(i+5));
    }

    plt::plot(ax0, z_x, z_y)->line_width(2);
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
