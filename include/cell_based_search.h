#ifndef CELL_BASED_SEARCH_H
#define CELL_BASED_SEARCH_H

#include <vector>

#include "cspace.h"
#include "graph.h"
#include "queue.h"

std::pair<Graph, cspace::Voronoi> cellBasedSearch(const cspace::state_t& x0, const cspace::state_t& xg,
                                                  const cspace::Options& opt, cspace::ReachedSet& R,
                                                  cspace::Voronoi& P);

std::pair<Graph, cspace::Voronoi> cellBasedSearch(const cspace::state_t& x0, const cspace::state_t& xg,
                                                  const cspace::Options& opt,
                                                  cspace::fun_inputs generateInput,
                                                  cspace::fun_simulator simulator);

std::pair<Graph, cspace::Voronoi> cellBasedSearch(const cspace::state_t& x0, const cspace::state_t& xg,
                                                  const cspace::Options& opt,
                                                  cspace::fun_motion_primitive primitives);

class MultiQuerySearch {
 public:
  MultiQuerySearch(std::size_t dim, const cspace::Options& opt, cspace::fun_motion_primitive primitives);
  std::pair<Graph, cspace::Voronoi> operator()(const cspace::state_t& x0, const cspace::state_t& xg);
 private:
  cspace::Options opt;
  cspace::fun_motion_primitive primitives;
  cspace::Voronoi P;
  cspace::ReachedSet R;
};

#endif  // CELL_BASED_SEARCH_H
