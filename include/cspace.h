#ifndef CSPACE_H
#define CSPACE_H
#include <KDTree.hpp>
#include <vector>
#include <random>

namespace cspace {

typedef std::vector<double> state_t;
typedef std::vector<double> input_t;
typedef std::vector<std::vector<double>> trajectory_t;
typedef std::vector<std::vector<double>> input_trajectory_t;
typedef std::shared_ptr<const state_t> state_ptr;
typedef std::shared_ptr<const input_trajectory_t> input_traj_ptr;

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
  using fun_dyn = std::function<cspace::state_t(const cspace::state_t&, const cspace::input_t&)>;
  using fun_reached = std::function<std::vector<cspace::input_trajectory_t>(const cspace::state_t&)>;

 public:
  struct InputTrajPtrTimePair;
  ReachedSet(fun_dyn dynamics, fun_reached motionPrimitive);
  void operator()(std::shared_ptr<const state_t> x0, const int time_steps, const double dt);
  bool empty();
  state_t pop_state();
  input_trajectory_t pop_input();
  std::shared_ptr<const state_t> pop_state_ptr();
  InputTrajPtrTimePair pop_input_ptr();
  state_t front();

 private:
  fun_dyn dynamics;
  fun_reached motionPrimitive;
  std::vector<std::shared_ptr<state_t>> reached_state_set;
  std::vector<std::shared_ptr<input_trajectory_t>> reached_input_set;
};

struct ReachedSet::InputTrajPtrTimePair {
  input_traj_ptr input;
  float time;
};

}  // namespace cspace
#endif
