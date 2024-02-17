#include "cell_based_search.h"

#include <vector>

#include "cspace.h"
#include "graph.h"
#include "queue.h"

Graph cellBasedSearch(
    const cspace::state_t& x0, const cspace::state_t& xg,
    std::function<cspace::state_t(const cspace::state_t&, const cspace::input_t&)> dynamics,
    std::function<std::vector<cspace::input_trajectory_t>(const cspace::state_t&)> motionPrimitive) {
  Graph G(x0);
  Queue Q(x0);
  cspace::Voronoi P(10000, x0, xg);
  cspace::ReachedSet R(dynamics, motionPrimitive);
  while (!Q.empty() && !P.target_reached()) {
    const auto [x_cur, cost] = Q.pop();
    int time_steps = 3;
    R(x_cur, time_steps, 0.1);
    while (!R.empty()) {
      if (P.visit(R.front())) {
        const cspace::state_t x = R.pop_state();
        const cspace::input_trajectory_t u = R.pop_input();
        Q.push(x, u, cost);
        G.add_vertex(x);
        G.add_edge(x_cur, x, u);
      }
    }
  }
  return G;
}
