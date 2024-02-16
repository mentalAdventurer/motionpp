#ifndef CSPACE_H
#define CSPACE_H
#include <KDTree.hpp>
#include <vector>

namespace cspace {

typedef std::vector<double> state_t;
typedef std::vector<double> input_t;
typedef std::vector<std::vector<double>> trajectory_t;
typedef std::vector<std::vector<double>> input_trajectory_t;

class Voronoi {
 public:
  Voronoi(const std::size_t N, state_t x0, state_t xg);
  ~Voronoi();
  bool visit(state_t x);
  bool target_reached();

 private:
  KDTree* kdtree;
  state_t state_limit;
  std::vector<state_t> points;
  std::vector<bool> points_visited;
  state_t random_state(const std::size_t state_dim);
  std::size_t xg_index;
};

class ReachedSet {
 public:
  ReachedSet(std::function<cspace::state_t(const cspace::state_t&, const cspace::input_t&)> dynamics,
             std::function<std::vector<cspace::input_trajectory_t>(const cspace::state_t&)> motionPrimitive);
  void operator()(state_t x0, int time_steps, double dt);
  bool empty();
  state_t pop_state();
  input_trajectory_t pop_input();
  state_t front();

 private:
  std::vector<state_t> reached_state_set;
  std::vector<input_trajectory_t> reached_input_set;
};

std::vector<double> linspace(double start, double end, int num);

}  // namespace cspace
#endif
