#include "cell_based_search.h"

#include <chrono>
#include <iostream>

std::pair<Graph, cspace::Voronoi> cellBasedSearch(const cspace::state_t& x0, const cspace::state_t& xg,
                                                  const cspace::Options& opt, cspace::ReachedSet& R,
                                                  cspace::Voronoi& P) {
  namespace cs = cspace;
  cs::state_ptr x0_ptr = std::make_shared<const cs::state_t>(x0);
  Graph G(x0_ptr);
  Queue Q(x0_ptr, opt.sort_metric);

  // Start Main Loop
  auto start = std::chrono::high_resolution_clock::now();
  while (!Q.empty() && !P.target_reached()) {
    auto [x_cur_ptr, cost] = Q.pop();
    R(x_cur_ptr);
    while (!R.empty() && !P.target_reached()) {
      auto x_ptr = R.pop_state_ptr();
      auto [u_ptr, time] = R.pop_input_ptr();
      if (P.visit(*x_ptr)) {
        Q.push(x_ptr, cost, u_ptr, time);
        G.add_vertex(x_ptr);
        G.add_edge(x_cur_ptr, x_ptr, u_ptr, time);
      }
    }
  }
  if (P.target_reached()) G.set_success(true);

  // Measure Time
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "Time taken by function: " << duration.count() << " ms" << std::endl;

  return {G, std::move(P)};
}

std::pair<Graph, cspace::Voronoi> cellBasedSearch(const cspace::state_t& x0, const cspace::state_t& xg,
                                                  const cspace::Options& opt,
                                                  cspace::fun_inputs generateInput,
                                                  cspace::fun_simulator simulator) {
  namespace cs = cspace;
  cs::Voronoi P(opt.NumberOfPoints, x0, xg, opt);
  cs::ReachedSet R(simulator, generateInput, opt);
  return cellBasedSearch(x0, xg, opt, R, P);
}

std::pair<Graph, cspace::Voronoi> cellBasedSearch(const cspace::state_t& x0, const cspace::state_t& xg,
                                                  const cspace::Options& opt,
                                                  cspace::fun_motion_primitive primitives) {
  namespace cs = cspace;
  cs::Voronoi P(opt.NumberOfPoints, x0, xg, opt);
  cs::ReachedSet R(primitives, opt);
  return cellBasedSearch(x0, xg, opt, R, P);
}
