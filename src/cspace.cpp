#include "cspace.h"

#include <algorithm>

using namespace cspace;

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
  std::mt19937 gen(rd());
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
ReachedSet::ReachedSet(fun_simulator simulator, fun_inputs generateInput)
    : simulator(simulator), generateInput(generateInput) {}

ReachedSet::ReachedSet(fun_motion_primitive primitives) : primitives(primitives) {}

void ReachedSet::operator()(const state_ptr x_ptr) {
  auto [inputs, time] = generateInput(*x_ptr);
  auto inputs_size = inputs.size();
  auto state_dim = x_ptr->size();
  auto num_primitive = time.size();
  auto traj_size = inputs_size / num_primitive / state_dim;

  // Motion Primitive Loop
  for (std::size_t i = 0; i < num_primitive; i++) {
    const auto dt = time[i] / traj_size;
    std::vector<double> x_traj = simulator(x_ptr->begin(), inputs.begin() + i * (state_dim * traj_size),
                                                state_dim, traj_size, dt);
    // Add if the state is not in collision
    if (!collision(x_traj, state_dim)) {
      add_reached_state(x_traj.end() - state_dim, state_dim);
      add_reached_input(inputs.begin() + i * (state_dim * traj_size), time[i], traj_size, state_dim);
    }
  }
}

bool ReachedSet::empty() { return states.empty(); }
bool ReachedSet::collision(std::vector<double> x, std::size_t state_dim) { return false; }

void ReachedSet::add_reached_state(std::vector<double>::const_iterator first, std::size_t states_dim) {
  state_ptr x_ptr(new state_t(first, first + states_dim));
  states.push_back(x_ptr);
}

void ReachedSet::add_reached_input(std::vector<double>::const_iterator first, float time,
                                   std::size_t traj_dim, std::size_t states_dim) {
  input_traj_ptr input_ptr(
      new input_trajectory_t(convertTo2D(first, first + traj_dim * states_dim, states_dim)));

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
    std::vector<double> row;
    row.reserve(state_dim);
    auto rowStart = first + i * state_dim;
    std::copy(rowStart, rowStart + state_dim, std::back_inserter(row));
    matrix.emplace_back(std::move(row));
  }

  return matrix;
}
