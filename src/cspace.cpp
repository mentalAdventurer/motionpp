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
