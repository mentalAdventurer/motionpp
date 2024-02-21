#include "cspace.h"

using namespace cspace;

Voronoi::Voronoi(const std::size_t N, state_t x0, state_t xg) {
  points.resize(N + 2);
  points[0] = x0;

  // Add the goal point to the points vector
  this->xg_index = 1;  // Store index for check in target_reached()
  points[xg_index] = xg;

  const std::size_t state_dim = x0.size();

  // TODO: Insert real state Limits
  for (std::size_t i = 0; i < state_dim; i++) {
    state_limit[i] = 10;
  }
  state_limit[0] = 1.2;
  state_limit[1] = 1.2;

  // Generate N random points
  for (std::size_t i = 0; i < N; i++) points[i + 2] = this->random_state(state_dim);

  // Create the kdTree
  kdtree = new KDTree(points);
}
Voronoi::~Voronoi() { delete kdtree; }
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
    std::uniform_real_distribution<> dis(-state_limit[i], state_limit[i]);
    randmo_vector[i] = dis(gen);
  }
  return randmo_vector;
}
bool Voronoi::target_reached() { return points_visited[xg_index]; }

// ReachedSet

ReachedSet::ReachedSet(fun_dyn dynamics, fun_reached motionPrimitive)
    : dynamics(dynamics), motionPrimitive(motionPrimitive) {}

void ReachedSet::operator()(const state_ptr& x_ptr) {
  auto [inputs, time] = motionPrimitive(*x_ptr);
  auto inputs_size = inputs.size();
  auto state_dim = x_ptr->size();
  auto num_primitive = time.size();
  auto traj_size = inputs_size / num_primitive / state_dim;

  // Motion Primitive Loop
  for (std::size_t i = 0; i < num_primitive; i++) {
    const auto dt = time[i] / traj_size;
    std::vector<double> x_traj = eulerIntegrate(x_ptr->begin(), inputs.begin() + i * (state_dim * traj_size),
                                                state_dim, traj_size, dt);
    // Add if the state is not in collision
    if (!collision(x_traj, state_dim)) {
      add_reached_state(x_traj.end() - state_dim, state_dim);
      add_reached_input(inputs.begin() + i * (state_dim * traj_size), time[i], traj_size, state_dim);
    }
  }
}

std::vector<double> ReachedSet::eulerIntegrate(std::vector<double>::const_iterator x0,
                                               std::vector<double>::const_iterator u_traj,
                                               std::size_t state_dim, std::size_t traj_dim, float dt) {
  std::vector<double> x_traj(traj_dim * state_dim);
  std::copy(x0, x0 + state_dim, x_traj.begin());

  for (std::size_t j = 0; j < traj_dim - 1; j++) {
    auto dxdt = dynamics(x_traj.begin() + j * state_dim, u_traj + j * state_dim, state_dim);
    for (std::size_t k = 0; k < state_dim; k++) {
      x_traj[(j + 1) * state_dim + k] = x_traj[j * state_dim + k] + dxdt[k] * dt;
    }
  }

  return x_traj;
}
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
