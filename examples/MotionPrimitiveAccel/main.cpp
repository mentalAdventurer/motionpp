#include <tuple>
#include <vector>

#include "cell_based_search.h"
#include "model.h"
#include "plot.h"


int main() {
  MotionPrimitives motion_primitives;
  cspace::state_t x = {0.1, 0.2, 0.4, 0.1, 0.3, 0.1};
  auto primitives = motion_primitives(x);
  auto [time, state, input] = primitives[2];

  // Plot
  plot_primitives(primitives);
  plot_states(time, state);
  plot_input(input, time.back());
  return 0;
}
