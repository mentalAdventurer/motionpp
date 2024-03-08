#include <matplot/freestanding/axes_functions.h>
#include <matplot/matplot.h>

#include <iostream>

#include "cell_based_search.h"
#include "inputShaping.h"
#include "model.h"

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

using primitiveTuple = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;
struct MotionPrimitive {
  std::vector<trajTuple> template_primitives;
  std::vector<trajTuple> template_primitives_neutral;
  //std::vector<double> speeds = {0.1, 0.2, 0.3, 0.4, 0.5};
  std::vector<double> speeds = {0.1, 0.3, 0.5};
  double max_speed = 0.5;
  MotionPrimitive();
  std::vector<trajTuple> operator()(const cspace::state_t x);
};

MotionPrimitive::MotionPrimitive() {
  static constexpr double start_position = 0.0, start_speed = 0.0;
  template_primitives.reserve(speeds.size());

  for (std::size_t i = 0; i < speeds.size(); i++) {
    template_primitives.push_back(get_zv_trajectory(start_position, start_speed, speeds[i]));
  }
  // Create Neutral Motion Primitive
  double time_step = param::dt;
  double time_end = 0.2;

  std::vector<double> time; time.reserve(time_end / time_step);
  for (double t = 0; t <= time_end; t += time_step) time.push_back(t);
  std::vector<double> traj_neutral(time.size() * 4, 0.0);
  std::vector<double> input_neutral(time.size(), 0.0);

  template_primitives_neutral.push_back(std::make_tuple(time, traj_neutral, input_neutral));
}

std::vector<trajTuple> MotionPrimitive::operator()(const cspace::state_t x) {
  std::size_t dim = x.size();
  std::vector<trajTuple> primitives;
  primitives.reserve(template_primitives.size());
  std::vector<double> traj;
  // Neutral Motion Primitive
  for (std::size_t i = 0; i < template_primitives_neutral.size(); i++) {
      for (std::size_t j = 0; j < std::get<1>(template_primitives_neutral[i]).size(); j += dim) {
        double t = std::get<0>(template_primitives_neutral[i])[j/4];
        traj.push_back(std::get<1>(template_primitives_neutral[i])[j]);
        traj.push_back(std::get<1>(template_primitives_neutral[i])[j + 1]);
        traj.push_back(std::get<1>(template_primitives_neutral[i])[j + 2] + x[2] + x[3] * t);
        traj.push_back(std::get<1>(template_primitives_neutral[i])[j + 3] + x[3]);
      }
      primitives.push_back(
          std::make_tuple(std::get<0>(template_primitives_neutral[i]), traj, std::get<2>(template_primitives_neutral[i])));
  }
  
  // Positve Motionprimitve
  for (std::size_t i = 0; i < speeds.size(); i++) {
    traj.clear();
    // No step bigger than max speed nor exceeding the max speed
    if (std::abs(speeds[i] + x[3]) <= max_speed) {
      for (std::size_t j = 0; j < std::get<1>(template_primitives[i]).size(); j += dim) {
        double t = std::get<0>(template_primitives[i])[j/4];
        traj.push_back(std::get<1>(template_primitives[i])[j]);
        traj.push_back(std::get<1>(template_primitives[i])[j + 1]);
        traj.push_back(std::get<1>(template_primitives[i])[j + 2] + x[2] + x[3] * t);
        traj.push_back(std::get<1>(template_primitives[i])[j + 3] + x[3]);
      }
      primitives.push_back(
          std::make_tuple(std::get<0>(template_primitives[i]), traj, std::get<2>(template_primitives[i])));
      traj.clear();
      }
    if (std::abs(-speeds[i] + x[3]) <= max_speed) {
      //Negative Motion Primitive 
      for (std::size_t j = 0; j < std::get<1>(template_primitives[i]).size(); j += dim) {
        double t = std::get<0>(template_primitives[i])[j/4];
        traj.push_back(-std::get<1>(template_primitives[i])[j]);
        traj.push_back(-std::get<1>(template_primitives[i])[j + 1]);
        traj.push_back(-std::get<1>(template_primitives[i])[j + 2] + x[2] + x[3] * t);
        traj.push_back(-std::get<1>(template_primitives[i])[j + 3] + x[3]);
      }
      std::vector<double> neg_input = std::get<2>(template_primitives[i]);
      std::transform(neg_input.begin(), neg_input.end(), neg_input.begin(), [](double u) { return -u; });
      primitives.push_back(
          std::make_tuple(std::get<0>(template_primitives[i]), traj, neg_input));
    }
  }
  return primitives;
}

int main() {
  MotionPrimitive primitives;

  cspace::state_t x0 = {0, 0, 0, 0};
  cspace::state_t xg = {0, 0, 1, 0.0};

  cspace::Options opt(1000, {
                               std::make_pair(0.0, 0.0),
                               std::make_pair(0.0, 0.0),
                               std::make_pair(-1, 1),
                               std::make_pair(-0.5, 0.5),
                           });


  cspace::state_t x_test = {0, 0, 0.1, 0.2};
  auto primitives_x0 = primitives(x_test);
  std::cout << "Number of primitives: " << primitives_x0.size() << std::endl;
  plot_states_and_input(std::get<0>(primitives_x0[1]), std::get<1>(primitives_x0[1]), std::get<2>(primitives_x0[1]));
  plot_primitives(primitives_x0);

  // Results
  auto [G, P] = cellBasedSearch(x0, xg, opt, primitives);
  std::cout << "Success: " << G.get_success() << std::endl;
  std::cout << "Number of Vertices: " << G.size_vertices() << std::endl;
  std::cout << "Initial State: " << G.front().state->at(0) << " " << G.front().state->at(1) << " "
            << G.front().state->at(2) << " " << G.front().state->at(3) << std::endl;
  std::cout << "Final State: " << G.back().state->at(0) << " " << G.back().state->at(1) << " "
            << G.back().state->at(2) << " " << G.back().state->at(3) << std::endl;
  plot_graph(G, x0, xg);
  plot_trajectory(G.get_trajectory(G.back().state));


  return 0;
}
