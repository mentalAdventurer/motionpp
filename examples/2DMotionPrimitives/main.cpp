#include <matplot/freestanding/axes_functions.h>
#include <matplot/matplot.h>

#include <iostream>

#include "cell_based_search.h"
#include "model.h"
#include "plot.h"

using primitiveTuple = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;

struct MotionPrimitive {
  std::vector<trajTuple> template_primitives;
  std::vector<double> speeds = {0.0 ,0.5,-0.5};
  std::vector<std::pair<double, double>> speeds_xy;
  double max_speed = 0.55;
  MotionPrimitive();
  std::vector<trajTuple> operator()(const cspace::state_t x);
  std::vector<trajTuple> generate_1d_primitives();
  void equalize_primitive_length(std::vector<trajTuple>& primitive1, std::vector<trajTuple>& primitive2);
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

std::vector<trajTuple> MotionPrimitive::generate_1d_primitives() {
  // Create Neutral Motion Primitive
  std::vector<trajTuple> template_primitives_1D;
  double time_step = param::dt;
  double time_end = 0.2;

  std::vector<double> time;
  time.reserve(time_end / time_step);
  for (double t = 0; t <= time_end; t += time_step) time.push_back(t);
  std::vector<double> traj_neutral(time.size() * 4, 0.0);
  std::vector<double> input_neutral(time.size(), 0.0);

  template_primitives_1D.push_back(std::make_tuple(time, traj_neutral, input_neutral));

  // Create Motion Primitives
  static constexpr double start_position = 0.0, start_speed = 0.0;
  template_primitives_1D.reserve(speeds.size());

  for (std::size_t i = 1; i < speeds.size(); i++) {
    template_primitives_1D.push_back(get_zv_trajectory(start_position, start_speed, speeds[i]));
  }
  return template_primitives_1D;
}

void equilize_primitive_length(trajTuple& primitive1, trajTuple& primitive2) {
  auto extend = [](std::vector<double>& time, std::vector<double>& traj, std::vector<double>& input,
                   size_t newSize) {
    if (time.empty()) return;  // Guard against empty vectors
    double lastTime = time.back(), timeStep = time.size() > 1 ? time[1] - time[0] : 1;
    for (size_t i = time.size(); i < newSize; ++i) {
      lastTime += timeStep;
      time.push_back(lastTime);
      input.push_back(0);               // Extend input with zeros
      if (traj.size() / 4 < newSize) {  // Extend traj if needed
        size_t lastIdx = traj.size() - 4;
        double x = traj[lastIdx] + traj[lastIdx + 1] * timeStep;
        double z = traj[lastIdx + 2] + traj[lastIdx + 3] * timeStep;
        traj.insert(traj.end(), {x, traj[lastIdx + 1], z, traj[lastIdx + 3]});  // Assuming constant velocity
      }
    }
  };

  size_t maxSize = std::max(std::get<0>(primitive1).size(), std::get<0>(primitive2).size());
  extend(std::get<0>(primitive1), std::get<1>(primitive1), std::get<2>(primitive1), maxSize);
  extend(std::get<0>(primitive2), std::get<1>(primitive2), std::get<2>(primitive2), maxSize);
}

MotionPrimitive::MotionPrimitive() {
  std::vector<trajTuple> template_primitives_1D = generate_1d_primitives();
  std::size_t index_x = 0;
  for (auto primitive_x : template_primitives_1D) {
    std::size_t index_y = 0;
    for (auto primitive_y : template_primitives_1D) {
      equilize_primitive_length(primitive_x, primitive_y);
      auto& [time_x, traj_x, input_x] = primitive_x;
      auto& [time_y, traj_y, input_y] = primitive_y;
      std::vector<double> traj;
      std::vector<double> input;
      for (std::size_t j = 0; j < time_x.size(); j++) {
        traj.push_back(traj_x[j * 4]);
        traj.push_back(traj_x[j * 4 + 1]);
        traj.push_back(traj_x[j * 4 + 2]);
        traj.push_back(traj_x[j * 4 + 3]);
        traj.push_back(traj_y[j * 4]);
        traj.push_back(traj_y[j * 4 + 1]);
        traj.push_back(traj_y[j * 4 + 2]);
        traj.push_back(traj_y[j * 4 + 3]);
        input.push_back(input_x[j]);
        input.push_back(input_y[j]);
      }
      speeds_xy.push_back({speeds[index_x], speeds[index_y]});
      template_primitives.push_back(std::make_tuple(time_x, traj, input));
      index_y++;
    }
    index_x++;
  }
}

std::vector<trajTuple> MotionPrimitive::operator()(const cspace::state_t x) {
  std::size_t dim = x.size();
  std::vector<trajTuple> primitives;
  primitives.reserve(template_primitives.size());

  std::size_t speed_index = 0;
  for (auto& [time_temp, traj_temp, input_temp] : template_primitives) {
    auto& [speed_x, speed_y] = speeds_xy[speed_index];
    double speed_squar = std::pow(speed_x + x[3], 2) + std::pow(speed_y + x[7], 2);
    if (speed_squar <= max_speed * max_speed) {
      std::vector<double> traj;
      for (std::size_t j = 0; j < traj_temp.size(); j += dim) {
        double t = time_temp[j / 8];
        traj.push_back(traj_temp[j]);
        traj.push_back(traj_temp[j + 1]);
        traj.push_back(traj_temp[j + 2] + x[2] + x[3] * t);
        traj.push_back(traj_temp[j + 3] + x[3]);
        traj.push_back(traj_temp[j + 4]);
        traj.push_back(traj_temp[j + 5]);
        traj.push_back(traj_temp[j + 6] + x[6] + x[7] * t);
        traj.push_back(traj_temp[j + 7] + x[7]);
      }
      primitives.push_back(std::make_tuple(time_temp, traj, input_temp));
    }
  }
  return primitives;
}

int main() {
  MotionPrimitive primitives;

  // Define Start and Goal
  cspace::state_t x0 = {0, 0, 0, 0, 0, 0, 0, 0};
  cspace::state_t xg = {0, 0, 0.2, 0, 0, 0, 0.2, 0};

  cspace::Options opt(20000, {
                                std::make_pair(0.0, 0.0),
                                std::make_pair(0.0, 0.0),
                                std::make_pair(-3, 3),
                                std::make_pair(-0.5, 0.5),
                                std::make_pair(0.0, 0.0),
                                std::make_pair(0.0, 0.0),
                                std::make_pair(-3, 3),
                                std::make_pair(-0.5, 0.5),
                            });
  // Define Obstacles
  // cspace::DynamicObstacle upper_cart({0.0, 0.0, 0, 0.01, 0, 0, 0.01, 0.01, 0, 0.0, 0.01, 0}, 4,
  // move_obstacle,
  //                                   x0);
  // cspace::StaticObstacle block({0.5, 0.0, 0, 0.51, 0, 0, 0.51, 0.01, 0, 0.5, 0.01, 0}, 4);
  // opt.dynamic_obstacles.push_back(upper_cart);
  // opt.static_obstacles.push_back(block);

  cspace::state_t x_test = {0, 0, 0.0, 0.1, 0, 0, 0.1, 0.1};
  auto primitives_x0 = primitives(x_test);
  std::cout << "Number of primitives: " << primitives_x0.size() << std::endl;
  //plot_states_and_input(std::get<0>(primitives_x0[1]), std::get<1>(primitives_x0[1]),
  //                      std::get<2>(primitives_x0[1]));
  plot_primitives(primitives_x0);

  // Results
  auto [G, P] = cellBasedSearch(x0, xg, opt, primitives);
  std::cout << "Success: " << G.get_success() << std::endl;
  std::cout << "Number of Vertices: " << G.size_vertices() << std::endl;
  std::cout << "Initial State: " << G.front().state->at(0) << " " << G.front().state->at(1) << " "
            << G.front().state->at(2) << " " << G.front().state->at(3) 
            << " " << G.front().state->at(4) << " " << G.front().state->at(5)
            << " " << G.front().state->at(6) << " " << G.front().state->at(7) << std::endl;
  std::cout << "Final State: " << G.back().state->at(0) << " " << G.back().state->at(1) << " "
            << G.back().state->at(2) << " " << G.back().state->at(3)
            << " " << G.back().state->at(4) << " " << G.back().state->at(5)
            << " " << G.back().state->at(6) << " " << G.back().state->at(7) << std::endl;
  plot_graph(G, x0, xg);
  trajectory_t traj = G.get_trajectory(G.back().state);
  auto [input, time] = G.get_input(G.back().state);
  auto traj_from_input = simulate_system(x0, input, param::dt);
  plot_trajectory(traj);
  //plot_trajectory_time(traj, time);
  //plot_trajectory_flat(traj_from_input, time);
  //plot_input(input, time);

  return 0;
}
