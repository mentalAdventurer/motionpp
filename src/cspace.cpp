#include "cspace.h"

#include <algorithm>

using namespace cspace;

DynamicObstacle::DynamicObstacle(const std::vector<double>& vertices, int num_vertices,
                                 decltype(transform_polytope) transform_fun, state_t x_cur)
    : polytope_data(vertices.begin(), vertices.end()),
      compatability_array(std::make_unique<double*[]>(vertices.size())),
      num_vertices(num_vertices),
      x_cur(x_cur),
      transform_polytope(transform_fun) {
  polytope.numpoints = num_vertices;
  for (std::size_t i = 0; i < vertices.size(); i++) compatability_array[i] = &polytope_data.data()[i];
  polytope.coord = compatability_array.get();
}

DynamicObstacle::DynamicObstacle(const DynamicObstacle& other)
    : polytope(other.polytope),
      polytope_data(other.polytope_data),
      compatability_array(std::make_unique<double*[]>(other.polytope_data.size())),
      num_vertices(other.num_vertices),
      x_cur(other.x_cur),
      transform_polytope(other.transform_polytope) {
  for (std::size_t i = 0; i < other.polytope_data.size(); i++)
    compatability_array[i] = &polytope_data.data()[i];
  polytope.coord = compatability_array.get();
}

DynamicObstacle& DynamicObstacle::operator=(const DynamicObstacle& other) {
  if (this != &other) {
    polytope = other.polytope;
    polytope_data = other.polytope_data;
    compatability_array = std::make_unique<double*[]>(other.polytope_data.size());
    num_vertices = other.num_vertices;
    x_cur = other.x_cur;
    transform_polytope = other.transform_polytope;
    for (std::size_t i = 0; i < other.polytope_data.size(); i++)
      compatability_array[i] = &polytope_data.data()[i];
    polytope.coord = compatability_array.get();
  }
  return *this;
}

DynamicObstacle::DynamicObstacle(DynamicObstacle&& other) noexcept
    : polytope(std::move(other.polytope)),
      polytope_data(std::move(other.polytope_data)),
      compatability_array(std::move(other.compatability_array)),
      num_vertices(other.num_vertices),
      x_cur(other.x_cur),
      transform_polytope(std::move(other.transform_polytope)) {}

DynamicObstacle& DynamicObstacle::operator=(DynamicObstacle&& other) noexcept {
  if (this != &other) {
    polytope = std::move(other.polytope);
    polytope_data = std::move(other.polytope_data);
    compatability_array = std::move(other.compatability_array);
    num_vertices = other.num_vertices;
    x_cur = other.x_cur;
    transform_polytope = std::move(other.transform_polytope);
  }
  return *this;
}

std::vector<double>& DynamicObstacle::transform_obstacle(std::vector<double>& vertices, const state_t& x0,
                                                         const state_t& xg) {
  vertices = transform_polytope(vertices, x0, xg);
  x_cur = xg;
  return vertices;
}

StaticObstacle::StaticObstacle(const std::vector<double>& vertices, int num_vertices)
    : polytope_data(vertices.begin(), vertices.end()),
      compatability_array(std::make_unique<double*[]>(vertices.size())),
      num_vertices(num_vertices) {
  polytope.numpoints = num_vertices;
  for (std::size_t i = 0; i < vertices.size(); i++) compatability_array[i] = &polytope_data.data()[i];
  polytope.coord = compatability_array.get();
}

StaticObstacle::StaticObstacle(const StaticObstacle& other)
    : polytope(other.polytope),
      polytope_data(other.polytope_data),
      compatability_array(std::make_unique<double*[]>(other.polytope_data.size())),
      num_vertices(other.num_vertices) {
  for (std::size_t i = 0; i < other.polytope_data.size(); i++)
    compatability_array[i] = &polytope_data.data()[i];
  polytope.coord = compatability_array.get();
}

StaticObstacle& StaticObstacle::operator=(const StaticObstacle& other) {
  if (this != &other) {
    polytope = other.polytope;
    polytope_data = other.polytope_data;
    compatability_array = std::make_unique<double*[]>(other.polytope_data.size());
    num_vertices = other.num_vertices;
    for (std::size_t i = 0; i < other.polytope_data.size(); i++)
      compatability_array[i] = &polytope_data.data()[i];
    polytope.coord = compatability_array.get();
  }
  return *this;
}

StaticObstacle::StaticObstacle(StaticObstacle&& other) noexcept
    : polytope(std::move(other.polytope)),
      polytope_data(std::move(other.polytope_data)),
      compatability_array(std::move(other.compatability_array)),
      num_vertices(other.num_vertices) {}

StaticObstacle& StaticObstacle::operator=(StaticObstacle&& other) noexcept {
  if (this != &other) {
    polytope = std::move(other.polytope);
    polytope_data = std::move(other.polytope_data);
    compatability_array = std::move(other.compatability_array);
    num_vertices = other.num_vertices;
  }
  return *this;
}

Voronoi::Voronoi(const std::size_t N, state_t x0, state_t xg, const Options::StateLimits& limits)
    : limits(limits) {
  points.resize(N + 2);
  points[0] = x0;

  // Add the goal point to the points vector
  this->xg_index = 1;  // Store index for check in target_reached()
  points[xg_index] = xg;

  const std::size_t state_dim = x0.size();

  // Generate N random points
  for (std::size_t i = 0; i < N; i++) points[i + 2] = this->random_state(state_dim);

  // Initialize the points_visited vector
  points_visited.resize(points.size());
  std::fill(points_visited.begin(), points_visited.end(), false);
  points_visited[0] = true;  // Mark the initial point as visited

  // Create the kdTree
  kdtree = std::make_unique<KDTree>(points);
}

Voronoi::Voronoi(Voronoi&& other) noexcept
    : kdtree(std::move(other.kdtree)),
      limits(std::move(other.limits)),
      points(std::move(other.points)),
      points_visited(std::move(other.points_visited)),
      xg_index(other.xg_index) {}

Voronoi& Voronoi::operator=(Voronoi&& other) noexcept {
  if (this != &other) {
    kdtree = std::move(other.kdtree);
    limits = other.limits;
    points = std::move(other.points);
    points_visited = std::move(other.points_visited);
    xg_index = other.xg_index;
  }
  return *this;
}

bool Voronoi::visit(state_t x) {
  // Find the nearest neighbor to 'x' using the kdTree
  int nearestIndex = kdtree->nearest_index(x);

  // Check if the point is already visited
  if (points_visited[nearestIndex]) {
    return false;
  }

  // Mark the nearest point as visited
  points_visited[nearestIndex] = true;
  return true;
}

state_t Voronoi::random_state(const std::size_t state_dim) {
  std::vector<double> randmo_vector(state_dim);
  std::random_device rd;
  static std::mt19937 gen(1111);
  for (std::size_t i = 0; i < state_dim; i++) {
    std::uniform_real_distribution<> dis(limits[i].first, limits[i].second);
    randmo_vector[i] = dis(gen);
  }
  return randmo_vector;
}
bool Voronoi::target_reached() { return points_visited[xg_index]; }
auto Voronoi::begin() -> decltype(points.begin()) { return points.begin(); }
auto Voronoi::end() -> decltype(points.end()) { return points.end(); }

// ReachedSet
ReachedSet::ReachedSet(fun_simulator simulator, fun_inputs generateInput, Options options)
    : simulator(simulator), generateInput(generateInput), opt(options) {}

ReachedSet::ReachedSet(fun_motion_primitive primitives, Options options)
    : primitives(primitives), opt(options) {}

void ReachedSet::operator()(const state_ptr x_ptr) {
  // Depeding on which constructor was used, call the appropriate initialization function
  if (primitives != nullptr) {
    init_reacheable_points_primitives(x_ptr);
  } else {
    init_reacheable_points_simulator(x_ptr);
  }
}

void ReachedSet::init_reacheable_points_simulator(const state_ptr x_ptr) {
  auto [inputs, time] = generateInput(*x_ptr);
  const std::size_t inputs_size = inputs.size();
  const std::size_t state_dim = x_ptr->size();
  const std::size_t num_primitive = time.size();
  const std::size_t traj_size = inputs_size / num_primitive / state_dim;

  // Motion Primitive Loop
  for (std::size_t i = 0; i < num_primitive; i++) {
    const auto dt = time[i] / traj_size;
    std::vector<double> x_traj =
        simulator(x_ptr->begin(), inputs.begin() + i * (state_dim * traj_size), state_dim, traj_size, dt);
    // Add if the state is not in collision
    if (!collision(x_traj.begin(), x_traj.end(), state_dim)) {
      add_reached_state(x_traj.end() - state_dim, state_dim);
      add_reached_input(inputs.begin() + i * (state_dim * traj_size), time[i], traj_size, state_dim);
    }
  }
}
void ReachedSet::init_reacheable_points_primitives(const state_ptr x_ptr) {
  auto primitives_for_x0 = primitives(*x_ptr);

  for (auto& [time, states, input] : primitives_for_x0) {
    if (!collision(states.begin(), states.end(), x_ptr->size())) {
      add_reached_state(states.end() - x_ptr->size(), x_ptr->size());
      add_reached_input(input.begin(), input.end(), time.back(), input.size() / time.size());
    }
  }
}

bool ReachedSet::empty() { return states.empty(); }

bool ReachedSet::collision(std::vector<double>::iterator begin, std::vector<double>::iterator end,
                           std::size_t state_dim) {
  if (opt.dynamic_obstacles.empty() || opt.static_obstacles.empty() || begin >= end) {
    return false;
  }

  for (; begin < end; begin += state_dim) {
    for (auto& dynamicObstacle : opt.dynamic_obstacles) {
      // Transform the dynamic obstacle's polytope data based on the current state
      dynamicObstacle.polytope_data =
          dynamicObstacle.transform_obstacle(dynamicObstacle.polytope_data, dynamicObstacle.x_cur,
                                             std::vector<double>(begin, begin + state_dim));

      for (auto& staticObstacle : opt.static_obstacles) {
        gkSimplex simplex;
        simplex.nvrtx = 0;
        auto distance = compute_minimum_distance(dynamicObstacle.polytope, staticObstacle.polytope, &simplex);

        // If distance indicates a collision, return true immediately
        if (distance <= 0) return true;
      }
    }
  }

  // No collision detected
  return false;
}

void ReachedSet::add_reached_state(std::vector<double>::const_iterator first,
                                   std::vector<double>::const_iterator last) {
  state_ptr x_ptr(new state_t(first, last));
  states.push_back(x_ptr);
}

void ReachedSet::add_reached_state(std::vector<double>::const_iterator first, std::size_t states_dim) {
  state_ptr x_ptr(new state_t(first, first + states_dim));
  states.push_back(x_ptr);
}

void ReachedSet::add_reached_input(std::vector<double>::const_iterator first,
                                   std::vector<double>::const_iterator last, float time,
                                   std::size_t input_dim) {
  input_traj_ptr input_ptr(new input_trajectory_t(first, last));
  times.push_back(time);
  input_traj.push_back(std::move(input_ptr));
}

void ReachedSet::add_reached_input(std::vector<double>::const_iterator first, float time,
                                   std::size_t traj_dim, std::size_t states_dim) {
  input_traj_ptr input_ptr(new input_trajectory_t(first, first + traj_dim * states_dim));

  times.push_back(time);
  input_traj.push_back(std::move(input_ptr));
}

std::shared_ptr<const state_t> ReachedSet::pop_state_ptr() {
  auto x_ptr = states.back();
  states.pop_back();
  return x_ptr;
}

InputTrajPtrTimePair ReachedSet::pop_input_ptr() {
  auto input_ptr = input_traj.back();
  input_traj.pop_back();
  auto time = times.back();
  times.pop_back();
  return std::make_pair(input_ptr, time);
}

state_t ReachedSet::front() { return *states.front(); }

std::size_t ReachedSet::size() { return states.size(); }

void ReachedSet::clear() {
  states.clear();
  input_traj.clear();
  times.clear();
}

std::vector<std::vector<double>> ReachedSet::convertTo2D(std::vector<double>::const_iterator first,
                                                         std::vector<double>::const_iterator last,
                                                         std::size_t state_dim) {
  std::size_t numRows = std::distance(first, last) / state_dim;
  std::vector<std::vector<double>> matrix;
  matrix.reserve(numRows);

  for (std::size_t i = 0; i < numRows; ++i) {
    auto rowStart = first + i * state_dim;
    // Construct a row directly within `matrix`, avoiding a temporary vector.
    // This uses the range constructor of std::vector to directly construct each row in place.
    matrix.emplace_back(rowStart, rowStart + state_dim);
  }

  return matrix;
}
