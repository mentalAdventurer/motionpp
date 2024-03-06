#include <matplot/freestanding/axes_functions.h>
#include <matplot/matplot.h>

#include <iostream>

#include "cell_based_search.h"
#include "inputShaping.h"
#include "model.h"

void plot_states_and_input(cvec_double& time, cvec_double& states, cvec_double& input,
                           std::size_t dim = 4) {
  namespace plt = matplot;

  // Repackage the states
  std::vector<double> x, xd, z, zd;

  for (std::size_t i = 4; i < states.size(); i += dim) {
    x.push_back(states[i]);
    xd.push_back(states[i + 1]);
    z.push_back(states[i + 2]);
    zd.push_back(states[i + 3]);
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


using primitiveTuple = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;
struct MotionPrimitive {
  std::vector<trajTuple> template_primitives;
  std::vector<double> speeds = {0.1, 0.2, 0.3, 0.4, 0.5};
  double max_speed = 0.5;
  MotionPrimitive();
  std::vector<trajTuple> operator()(const cspace::state_t x);
};

MotionPrimitive::MotionPrimitive() {
  double start_position = 0.0, start_speed = 0.0;
  template_primitives.reserve(speeds.size());

  for (std::size_t i = 0; i < speeds.size(); i++) {
    template_primitives.push_back(get_zv_trajectory(start_position, start_speed, speeds[i]));
  }
}

std::vector<trajTuple> MotionPrimitive::operator()(const cspace::state_t x) {
  std::size_t dim = x.size();
  std::vector<trajTuple> primitives; primitives.reserve(template_primitives.size());
  // Positve Motionprimitve
  for (std::size_t i = 0; i < speeds.size(); i++) {
    // No step bigger than max speed nor exceeding the max speed
    if (speeds[i] + std::abs(x[3]) <= max_speed) {
      double t0 = std::get<0>(template_primitives[i])[0];
      double t1 = std::get<0>(template_primitives[i])[1];
      double dt = t1 - t0; 
      std::vector<double> traj;
      for(std::size_t j = 0; j < std::get<1>(template_primitives[i]).size(); j+=dim){
        traj.push_back(std::get<1>(template_primitives[i])[j]);
        traj.push_back(std::get<1>(template_primitives[i])[j+1]);
        traj.push_back(std::get<1>(template_primitives[i])[j+2]+x[2]+x[3]*dt);
        traj.push_back(std::get<1>(template_primitives[i])[j+3]+x[3]);
      }
      primitives.push_back(std::make_tuple(std::get<0>(template_primitives[i]), traj, std::get<2>(template_primitives[i])));
    }
  }
  return primitives;
}

int main() {
  MotionPrimitive primitives;

  cspace::state_t x0 = {0, 0, 0.1, 0};
  auto primitives_x0 = primitives(x0);
  auto [time, states, input] = primitives_x0[0]; 
  std::cout << "Amount of Primitive: " << primitives_x0.size() << std::endl;
  plot_states_and_input(time, states, input);

  return 0;
}
