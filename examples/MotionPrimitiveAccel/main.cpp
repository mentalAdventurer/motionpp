#include <tuple>
#include <vector>

#include "cell_based_search.h"
#include "model.h"
#include "plot.h"

// Define Metric (Optional)
// Used for choosing next vertex for search
float metric(const std::shared_ptr<const std::vector<double>>& x,
             const std::shared_ptr<const std::vector<double>>& u, const float& time) {
  std::vector<double> xg = {0.2, 0, 0.0, 0.2, 0, 0 };
  float timeCoeff = 0.5;
  float inputCoeff = 0.01;
  float distanceCoeff = 0.09;

  // Calculate sum of the absolute values of u
  float absSumU = std::transform_reduce(
      u->begin(), u->end(),
      0.0,                                     // Initial value
      std::plus<>(),                           // Summation operation
      [](float val) { return std::abs(val); }  // Transformation operation (absolute value)
  );
  float meanAbsU = absSumU / u->size();

  // Distance to goal
  float distance = std::sqrt(std::pow(x->at(0) - xg[0], 2) + std::pow(x->at(3) - xg[3], 2));

  // Calculate the final cost
  float cost = timeCoeff * time + inputCoeff * meanAbsU + distanceCoeff * distance;

  return cost;
};

// Define movement of obstacle to state 
std::vector<std::vector<double>>& move_obstacle(std::vector<std::vector<double>>& obstacle, const cspace::state_t& x0,
                                   const cspace::state_t& xg) {
  double distance_x = xg[0] - x0[0];
  double distance_y = xg[3] - x0[3];

  // 2D Movement
  for(auto& vertex : obstacle) {
    vertex[0] += distance_x;
    vertex[1] += distance_y;
  }

  return obstacle;
}

int main() {
  MotionPrimitives motion_primitives;
  cspace::state_t x = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  auto primitives = motion_primitives(x);
  auto [time, state, input] = primitives[2];

  // Define Start and Goal
  std::vector<double> x0 = {0, 0, 0.0, 0.2, 0, 0};
  std::vector<double> xg = {0.2, 0, 0, 0.2, 0, 0};

  // Define Objects
  cspace::DynamicObstacle upper_cart({{x0[0], x0[3], 0.0}}, move_obstacle, x0);
  cspace::StaticObstacle block({{0.05, -0.05, 0}, {0.15, -0.05, 0}, {0.15, 0.3, 0}, {0.05, 0.3, 0}});
  // Define Search Options
  cspace::Options opt(7e3,  // Number of Voronoi points
                      {
                          // Region for spreading Voronoi points
                          {-0.5, 0.5},  // Position x
                          {0.0, 0.0},   // Speed x
                          {0.0, 0.0},   // Accel x
                          {-0.5, 0.5},  // Position y
                          {0.0, 0.0},   // Speed y
                          {0.0, 0.0}    // Accel y
                      });
  opt.sort_metric = metric;
  opt.dynamic_obstacles.push_back(upper_cart);
  opt.static_obstacles.push_back(block);

  // Start search
  auto [G, P] = cellBasedSearch(x0, xg, opt, motion_primitives);
  auto trajectory = G.get_trajectory(G.back().state);

  // Terminal Info
  std::cout << "Numer of Primitives: " << primitives.size() << std::endl;
  (G.get_success()) ? std::cout << "Success" << std::endl : std::cout << "Failure" << std::endl;
  std::cout << "Number of Vertices: " << G.size_vertices() << std::endl;
  std::cout << "Final State: ";
  for (auto& xi : *(G.back().state)) std::cout << xi << " ";
  std::cout << std::endl;

  // Plot MotionPrimitives
  //plot_primitives(primitives);
  //plot_states(time, state);
  //plot_input(input, time.back());

  // Plot Graph
  plot_graph(G,x0,xg,trajectory,opt.static_obstacles);

  return 0;
}
