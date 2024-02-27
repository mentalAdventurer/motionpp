#include "cell_based_search.h"

#include <iostream>



std::pair<Graph,cspace::Voronoi> cellBasedSearch(const cspace::state_t& x0, const cspace::state_t& xg, cspace::fun_dyn dynamics,
                      cspace::fun_reached motionPrimitive, const cspace::Options& opt) {
  namespace cs = cspace;
  cs::state_ptr x0_ptr = std::make_shared<const cs::state_t>(x0);
  Graph G(x0_ptr);
  Queue Q(x0_ptr);
  cs::Voronoi P(opt.NumberOfPoints, x0, xg, opt.limits);
  cs::ReachedSet R(dynamics, motionPrimitive);
  while (!Q.empty() && !P.target_reached()) {
    const auto [x_cur_ptr, cost] = Q.pop();
    R(x_cur_ptr);
    while (!R.empty()) {
      auto x_ptr = R.pop_state_ptr();
      auto [u_ptr, time] = R.pop_input_ptr();
      if (P.visit(*x_ptr)) {
        Q.push(x_ptr, cost, u_ptr, time);
        G.add_vertex(x_ptr);
        G.add_edge(x_cur_ptr, x_ptr, u_ptr, time);
      }
    }
  }
  if (P.target_reached()) {
    G.set_success(true);
  }
  return {G,P};
}
