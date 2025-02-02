#include <tuple>
#include <vector>

#include "cell_based_search.h"
#include "model.h"
#include "plot.h"
#include "helper.h"


int main() {
  // Generate Motion Primitives function with mphelper
  std::vector<std::vector<double>> inputs;
  double step_size = 2*param::ddphi_max/10; 
  for (double i = -param::ddphi_max; i <= param::ddphi_max; i += step_size) {
    inputs.push_back({i});
  }
  auto mp = mphelper::Euler(model::system, inputs,0.08);
  auto primitives = mp({0, 0, 0, 0});
  //plot_primitives(primitives);


  // Define Search Options
  cspace::Options opt(66000,  // Number of Voronoi points
                      {
                          // Region for spreading Voronoi points
                          {0.0, 0.0},  // Position x
                          {0.0, 0.0},   // Speed x
                          {-2*param::pi, 2*param::pi},   // Accel x
                          {-30, 30}
                      });

  opt.target_radius = 0.10;
  opt.distance_metric = [](const std::vector<double>& x1, const std::vector<double>& x2){return std::abs(x1[2]-x2[2])+std::abs(x1[3]-x2[3]);};
  std::vector<double> x0 = {0, 0, 0, 0};
  std::vector<double> xg = {0, 0, param::pi, 0};

  // Start search
  MultiQuerySearch cellBasedSearch(x0.size(), opt, mp);
  auto G = cellBasedSearch(x0, xg);

  auto trajectory = G.get_trajectory(G.back().state);
  auto [input,t] = G.get_input(G.back().state);

  // Terminal Info
  std::cout << "Numer of Primitives: " << primitives.size() << std::endl;
  (G.get_success()) ? std::cout << "Success" << std::endl : std::cout << "Failure" << std::endl;
  std::cout << "Number of Vertices: " << G.size_vertices() << std::endl;
  std::cout << "Final State: ";
  for (auto& xi : *(G.back().state)) std::cout << xi << " ";
  std::cout << "\nDistance: " << opt.distance_metric(*(G.back().state),xg) << "\n";
  std::cout << std::endl;

  // Create Time vector
  std::vector<double> time(trajectory.size(), 0); 
  step_size = t/time.size();
  for(std::size_t i = 0; i < time.size(); i++) time[i] = i * step_size;


  plot_trajectory(time,trajectory);

  return 0;
}
