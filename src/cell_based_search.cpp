#include "cell_based_search.h"

#include <vector>

#include "cspace.h"
#include "graph.h"
#include "queue.h"

Graph cellBasedSearch(
    const cspace::state_t& x0, const cspace::state_t& xg,
    std::function<cspace::state_t(const cspace::state_t&, const cspace::input_t&)> dynamics,
    std::function<std::vector<cspace::input_trajectory_t>(const cspace::state_t&)> motionPrimitive) {
    
  namespace cs = cspace;
  cs::state_ptr x0_ptr = std::make_shared<const cs::state_t>(x0);
  Graph G(x0_ptr);
  Queue Q(x0_ptr);
  cs::Voronoi P(10000, x0, xg);
  cs::ReachedSet R(dynamics, motionPrimitive);
  while (!Q.empty() && !P.target_reached()) {
    const auto [x_cur_ptr, cost] = Q.pop();
    int time_steps = 3;
    R(x_cur_ptr, time_steps, 0.1);
    while (!R.empty()) {
      if (P.visit(R.front())) {
        auto x_ptr = R.pop_state_ptr();
        auto u_ptr = R.pop_input_ptr();
        Q.push(x_ptr, u_ptr, cost);
        G.add_vertex(x_ptr);
        G.add_edge(x_cur_ptr, x_ptr, u_ptr);
      }
    }
  }
  return G;
}
