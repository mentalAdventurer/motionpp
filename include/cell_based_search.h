#ifndef CELL_BASED_SEARCH_H
#define CELL_BASED_SEARCH_H

#include <vector>

#include "cspace.h"
#include "graph.h"
#include "queue.h"

std::pair<Graph, cspace::Voronoi> cellBasedSearch(const cspace::state_t& x0, const cspace::state_t& xg,
                                                  const cspace::Options& opt,
                                                  cspace::fun_inputs generateInput,
                                                  cspace::fun_simulator simulator); 


#endif  // CELL_BASED_SEARCH_H
