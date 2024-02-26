#ifndef CSPACE_H
#define CSPACE_H
#include <KDTree.hpp>
#include <random>
#include <vector>

namespace cspace {

typedef std::vector<double> state_t;
typedef std::vector<double> input_t;
typedef std::vector<std::vector<double>> trajectory_t;
typedef std::vector<std::vector<double>> input_trajectory_t;
typedef std::shared_ptr<const state_t> state_ptr;
typedef std::shared_ptr<input_trajectory_t> input_traj_ptr;
using InputTrajPtrTimePair = std::pair<input_traj_ptr, float>;
using fun_dyn = std::function<cspace::state_t(std::vector<double>::const_iterator,
                                              std::vector<double>::const_iterator, std::size_t)>;
using fun_reached = std::function<std::pair<std::vector<double>, std::vector<float>>(const cspace::state_t&)>;

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
  ReachedSet(fun_dyn dynamics, fun_reached motionPrimitive);
  void operator()(const state_ptr x_ptr);
  bool empty();
  std::shared_ptr<const state_t> pop_state_ptr();
  InputTrajPtrTimePair pop_input_ptr();
  state_t front();
  void clear();
  std::size_t size();

 private:
  fun_dyn dynamics;
  fun_reached motionPrimitive;
  void simulate(const state_ptr& x0, std::pair<std::vector<double>, std::vector<float>>& InputTimePair);
  bool collision(std::vector<double> x, std::size_t state_dim);
  void add_reached_state(std::vector<double>::const_iterator first, std::size_t states_dim);
  void add_reached_input(std::vector<double>::const_iterator first, float time, std::size_t traj_dim,
                         std::size_t states_dim);
  std::vector<std::vector<double>> convertTo2D(std::vector<double>::const_iterator first,
                                               std::vector<double>::const_iterator last,
                                               std::size_t state_dim);
  std::vector<double> eulerIntegrate(std::vector<double>::const_iterator x0, std::vector<double>::const_iterator u_traj,
                                     std::size_t state_dim, std::size_t traj_dim, float dt);
  std::vector<state_ptr> states;
  std::vector<float> times;
  std::vector<input_traj_ptr> input_traj;
};

}  // namespace cspace
#endif
