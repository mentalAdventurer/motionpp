#include "cell_based_search.h"

#include <vector>

#include "cspace.h"
#include "graph.h"
#include "queue.h"

Graph cellBasedSearch(
    const cspace::state_t& x0, const cspace::state_t& xg,
    std::function<cspace::state_t(const cspace::state_t&, const cspace::input_t&)> dynamics,
    std::function<std::vector<cspace::input_trajectory_t>(const cspace::state_t&)> motionPrimitive) {

  std::shared_ptr<const cspace::state_t> x0_ptr = std::make_shared<const cspace::state_t>(x0);
  Graph G(x0_ptr);
  Queue Q(x0_ptr);
  cspace::Voronoi P(10000, x0, xg);
  cspace::ReachedSet R(dynamics, motionPrimitive);
  while (!Q.empty() && !P.target_reached()) {
    const auto [x_cur, cost] = Q.pop();
    int time_steps = 3;
    R(x_cur, time_steps, 0.1);
    while (!R.empty()) {
      if (P.visit(R.front())) {
        auto x = R.pop_state_ptr();
        auto u = R.pop_input_ptr();
        Q.push(x, u, cost);
        G.add_vertex(x);
        G.add_edge(x_cur, x, u);
      }
    }
  }
  return G;
}
