#ifndef CSPACE_H
#define CSPACE_H
#include <KDTree.hpp>
#include <random>
#include <vector>

#include "annoylib.h"
#include "kissrandom.h"
extern "C" {
#include <openGJK/openGJK.h>
}

namespace cspace {

typedef std::vector<double> state_t;
typedef std::vector<double> input_t;
typedef std::vector<std::vector<double>> trajectory_t;
typedef std::vector<double> input_trajectory_t;
typedef std::shared_ptr<const state_t> state_ptr;
using input_traj_ptr = std::shared_ptr<const input_trajectory_t>;
using InputTrajPtrTimePair = std::pair<input_traj_ptr, float>;
using trajTuple = std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>;
using fun_simulator =
    std::function<std::vector<double>(std::vector<double>::const_iterator,
                                      std::vector<double>::const_iterator, std::size_t, std::size_t, float)>;
using fun_inputs = std::function<std::pair<std::vector<double>, std::vector<float>>(const cspace::state_t&)>;
using fun_motion_primitive = std::function<std::vector<trajTuple>(const state_t&)>;

struct DynamicObstacle {
  gkPolytope polytope;
  std::vector<double> polytope_data;
  std::unique_ptr<double*[]> compatability_array;
  int num_vertices;
  state_t x_cur;
  std::function<std::vector<double>&(std::vector<double>&, const state_t&, const state_t&)>
      transform_polytope;
  DynamicObstacle(const std::vector<double>& vertices, int num_vertices,
                  decltype(transform_polytope) transform_fun, state_t x_cur);
  ~DynamicObstacle(){};
  DynamicObstacle(const DynamicObstacle& other);
  DynamicObstacle& operator=(const DynamicObstacle& other);
  DynamicObstacle(DynamicObstacle&& other) noexcept;
  DynamicObstacle& operator=(DynamicObstacle&& other) noexcept;
  std::vector<double>& transform_obstacle(std::vector<double>&, const state_t& x0, const state_t& xg);
};

struct StaticObstacle {
  gkPolytope polytope;
  std::vector<double> polytope_data;
  std::unique_ptr<double*[]> compatability_array;
  int num_vertices;
  ~StaticObstacle(){};
  StaticObstacle(const std::vector<double>& vertices, int num_vertices);
  StaticObstacle(const StaticObstacle& other);
  StaticObstacle& operator=(const StaticObstacle& other);
  StaticObstacle(StaticObstacle&& other) noexcept;
  StaticObstacle& operator=(StaticObstacle&& other) noexcept;
};

struct Options {
  using StateLimits = std::vector<std::pair<double, double>>;
  using metric_func =
      std::function<float(const cspace::state_ptr&, const cspace::input_traj_ptr&, const float&)>;
  std::size_t NumberOfPoints;
  StateLimits limits;
  metric_func sort_metric;
  std::vector<DynamicObstacle> dynamic_obstacles;
  std::vector<StaticObstacle> static_obstacles;
  Options(std::size_t NumberOfPoints, StateLimits limits)
      : NumberOfPoints(NumberOfPoints), limits(limits), sort_metric(nullptr) {}
};

class Voronoi {
 private:
  Annoy::AnnoyIndex<int, double, Annoy::Euclidean, Annoy::Kiss32Random,
                    Annoy::AnnoyIndexSingleThreadedBuildPolicy>
      tree;
  std::unique_ptr<KDTree> kdtree;
  Options::StateLimits limits;
  std::vector<state_t> points;
  std::vector<bool> points_visited;
  state_t random_state(const std::size_t state_dim);
  std::size_t xg_index;

 public:
  Voronoi(const std::size_t N, state_t x0, state_t xg, const Options::StateLimits& limits);
  Voronoi(const Voronoi&) = delete;
  Voronoi& operator=(const Voronoi&) = delete;
  Voronoi(Voronoi&& other) noexcept;
  Voronoi& operator=(Voronoi&& other) noexcept;
  bool visit(state_t x);
  bool target_reached();
  auto begin() -> decltype(points.begin());
  auto end() -> decltype(points.end());
  int search_k = 1;
  int n_trees = 100;
};

class ReachedSet {
 public:
  ReachedSet(fun_simulator simulator, fun_inputs generateInput, Options options);
  ReachedSet(fun_motion_primitive primitives, Options options);
  void operator()(const state_ptr x_ptr);
  void init_reacheable_points_simulator(const state_ptr x_ptr);
  void init_reacheable_points_primitives(const state_ptr x_ptr);
  std::shared_ptr<const state_t> pop_state_ptr();
  InputTrajPtrTimePair pop_input_ptr();
  state_t front();
  void clear();
  bool empty();
  std::size_t size();

 private:
  fun_simulator simulator = nullptr;
  fun_inputs generateInput = nullptr;
  fun_motion_primitive primitives = nullptr;
  Options opt;
  bool collision(std::vector<double>::iterator, std::vector<double>::iterator, std::size_t);
  void add_reached_state(std::vector<double>::const_iterator first, std::size_t states_dim);
  void add_reached_state(std::vector<double>::const_iterator first, std::vector<double>::const_iterator last);
  void add_reached_input(std::vector<double>::const_iterator first, float time, std::size_t traj_dim,
                         std::size_t states_dim);
  void add_reached_input(std::vector<double>::const_iterator first, std::vector<double>::const_iterator last,
                         float time, std::size_t input_dim);
  std::vector<std::vector<double>> convertTo2D(std::vector<double>::const_iterator first,
                                               std::vector<double>::const_iterator last,
                                               std::size_t state_dim);
  std::vector<double> eulerIntegrate(std::vector<double>::const_iterator x0,
                                     std::vector<double>::const_iterator u_traj, std::size_t state_dim,
                                     std::size_t traj_dim, float dt);
  std::vector<state_ptr> states;
  std::vector<float> times;
  std::vector<input_traj_ptr> input_traj;
};

}  // namespace cspace
#endif
