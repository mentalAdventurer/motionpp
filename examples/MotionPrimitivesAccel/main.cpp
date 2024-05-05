#include <tuple>
#include <vector>

#include "cell_based_search.h"
#include "model.h"
#include "plot.h"

// Define Metric (Optional)
// Used for choosing next vertex for search
float metric(const std::shared_ptr<const std::vector<double>>& x,
             const std::shared_ptr<const std::vector<double>>& u, const float& time) {
  std::vector<double> xg = {0.2, 0, 0.0};
  float timeCoeff = 0.0;
  float inputCoeff = 0.00;
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
  float distance = x->at(0) - xg[0];

  // Calculate the final cost
  float cost = timeCoeff * time + inputCoeff * meanAbsU + distanceCoeff * distance;

  return cost;
};

// Define movement of obstacle to state 
std::vector<std::vector<double>>& move_obstacle(std::vector<std::vector<double>>& obstacle, const cspace::state_t& x0,
                                   const cspace::state_t& xg) {
  double distance = xg[0] - x0[0];

  // 2D Movement
  for(auto& vertex : obstacle) {
    vertex[0] += distance;
  }

  return obstacle;
}

int main() {
  MotionPrimitives motion_primitives;
  cspace::state_t x = {0.1, 0.1, 0.1};
  auto primitives = motion_primitives(x);
  auto [time, state, input] = primitives[2];

  // Define Start and Goal
  std::vector<double> x0 = {0, 0, 0.0};
  std::vector<double> xg = {0.3, 0, 0};

  // Define Search Options
  cspace::Options opt(1e4,  // Number of Voronoi points
                      {
                          // Region for spreading Voronoi points
                          {0.0, 0.5},  // Position x
                          {0.0, 0.5},   // Speed x
                          {0.0, 0},   // Accel x
                      });
  opt.sort_metric = metric;

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
  plot_primitives(primitives);
  //plot_states(time, state);
  //plot_input(input, time.back());

  // Plot Graph
  plot_graph(G,x0,xg,trajectory);

  return 0;
}
