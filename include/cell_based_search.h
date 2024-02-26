#ifndef CELL_BASED_SEARCH_H
#define CELL_BASED_SEARCH_H

#include <vector>
#include "cspace.h"
#include "graph.h"
#include "queue.h"


Graph cellBasedSearch(const cspace::state_t& x0, const cspace::state_t& xg, cspace::fun_dyn dynamics,
                      cspace::fun_reached motionPrimitive);

#endif // CELL_BASED_SEARCH_H
