#include <matplot/freestanding/axes_functions.h>
#include <matplot/matplot.h>

#include <iostream>

#include "cell_based_search.h"
#include "model.h"
#include "plot.h"

using primitiveTuple = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;

struct MotionPrimitive {
  std::vector<trajTuple> template_primitives;
  std::vector<trajTuple> template_primitives_neutral;
  // std::vector<double> speeds = {0.1, 0.2, 0.3, 0.4, 0.5};
  std::vector<double> speeds = {0.0,0.1,-0.1,0.3,-0.3,0.5,-0.5};
  double max_speed = 0.55;
  MotionPrimitive();
  std::vector<trajTuple> operator()(const cspace::state_t x);
};


std::vector<double>& move_obstacle(std::vector<double>& obstacle, const cspace::state_t& x0,
                                  const cspace::state_t& xg) {
  double distance = xg[2] - x0[2];

  // 1D Movement
  for (std::size_t i = 0; i < obstacle.size(); i += 3) {
    obstacle[i] += distance;
  }

  return obstacle;
}

MotionPrimitive::MotionPrimitive() {
  // Create Neutral Motion Primitive
  double time_step = param::dt;
  double time_end = 0.2;

  std::vector<double> time;
  time.reserve(time_end / time_step);
  for (double t = 0; t <= time_end; t += time_step) time.push_back(t);
  std::vector<double> traj_neutral(time.size() * 4, 0.0);
  std::vector<double> input_neutral(time.size(), 0.0);

  template_primitives.push_back(std::make_tuple(time, traj_neutral, input_neutral));

  // Create Motion Primitives
  static constexpr double start_position = 0.0, start_speed = 0.0;
  template_primitives.reserve(speeds.size());

  for (std::size_t i = 1; i < speeds.size(); i++) {
    template_primitives.push_back(get_zv_trajectory(start_position, start_speed, speeds[i]));
  }
}

std::vector<trajTuple> MotionPrimitive::operator()(const cspace::state_t x) {
  std::size_t dim = x.size();
  std::vector<trajTuple> primitives;
  primitives.reserve(template_primitives.size());

  for (std::size_t i = 0; i < template_primitives.size(); i++) {
    std::vector<double> traj;
    // No step bigger than max speed nor exceeding the max speed
    if (std::abs(speeds[i] + x[3]) <= max_speed) {
      for (std::size_t j = 0; j < std::get<1>(template_primitives[i]).size(); j += dim) {
        double t = std::get<0>(template_primitives[i])[j / 4];
        traj.push_back(std::get<1>(template_primitives[i])[j]);
        traj.push_back(std::get<1>(template_primitives[i])[j + 1]);
        traj.push_back(std::get<1>(template_primitives[i])[j + 2] + x[2] + x[3] * t);
        traj.push_back(std::get<1>(template_primitives[i])[j + 3] + x[3]);
      }
      primitives.push_back(
          std::make_tuple(std::get<0>(template_primitives[i]), traj, std::get<2>(template_primitives[i])));
    }
  }
  return primitives;
}

int main() {
  MotionPrimitive primitives;

  // Define Start and Goal
  cspace::state_t x0 = {0, 0, 0, 0};
  cspace::state_t xg = {0, 0, 1, 0.0};

  cspace::Options opt(2000, {
                                std::make_pair(0.0, 0.0),
                                std::make_pair(0.0, 0.0),
                                std::make_pair(-3, 3),
                                std::make_pair(-0.5, 0.5),
                            });
  // Define Obstacles
  cspace::DynamicObstacle upper_cart({0.0, 0.0, 0, 0.01, 0, 0, 0.01, 0.01, 0, 0.0, 0.01, 0}, 4, move_obstacle,x0);
  cspace::StaticObstacle block({0.5, 0.0, 0, 0.51, 0, 0, 0.51, 0.01, 0, 0.5, 0.01, 0}, 4);
  //opt.dynamic_obstacles.push_back(upper_cart);
  //opt.static_obstacles.push_back(block);

  cspace::state_t x_test = {0, 0, 0.0, 0.1};
  auto primitives_x0 = primitives(x_test);
  std::cout << "Number of primitives: " << primitives_x0.size() << std::endl;
  plot_states_and_input(std::get<0>(primitives_x0[1]), std::get<1>(primitives_x0[1]),
                       std::get<2>(primitives_x0[1]));
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
  trajectory_t traj = G.get_trajectory(G.back().state);
  auto [input,time] = G.get_input(G.back().state);
  auto traj_from_input = simulate_system({0, 0, 0, 0}, input, param::dt);
  plot_trajectory(traj);
  plot_trajectory_time(traj, time);
  plot_trajectory_flat(traj_from_input, time);
  plot_input(input, time);

  return 0;
}
