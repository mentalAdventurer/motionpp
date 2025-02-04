#include "cspace.h"

#include <algorithm>

using namespace cspace;

DynamicObstacle::DynamicObstacle(const std::vector<std::vector<double>>& vertices,
                                 decltype(transform_polytope) transform_fun, state_t x_cur)
    : StaticObstacle(vertices), x_cur(x_cur), transform_polytope(transform_fun) {}

std::vector<std::vector<double>>& DynamicObstacle::transform_obstacle(
    std::vector<std::vector<double>>& vertices, const state_t& x0, const state_t& xg) {
  vertices = transform_polytope(vertices, x0, xg);
  x_cur = xg;
  return vertices;
}

StaticObstacle::StaticObstacle(const std::vector<std::vector<double>>& vertices)
    : polytope_data(vertices.begin(), vertices.end()),
      compatability_array(std::make_unique<double*[]>(vertices.size())),
      num_vertices(vertices.size()) {
  polytope.numpoints = num_vertices;
  for (std::size_t i = 0; i < vertices.size(); i++) compatability_array[i] = polytope_data[i].data();
  polytope.coord = compatability_array.get();
}

StaticObstacle::StaticObstacle(const StaticObstacle& other)
    : polytope(other.polytope),
      polytope_data(other.polytope_data),
      compatability_array(std::make_unique<double*[]>(other.polytope_data.size())),
      num_vertices(other.num_vertices) {
  for (std::size_t i = 0; i < polytope_data.size(); i++) compatability_array[i] = polytope_data[i].data();
  polytope.coord = compatability_array.get();
}

StaticObstacle& StaticObstacle::operator=(const StaticObstacle& other) {
  if (this != &other) {
    polytope = other.polytope;
    polytope_data = other.polytope_data;
    compatability_array = std::make_unique<double*[]>(other.polytope_data.size());
    num_vertices = other.num_vertices;
    for (std::size_t i = 0; i < polytope_data.size(); i++) compatability_array[i] = polytope_data[i].data();
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

void Voronoi::initialize(const std::size_t N, std::size_t state_dim) {

  // Initialize the points_visited vector
  points_visited.resize(points.size());
  std::fill(points_visited.begin(), points_visited.end(), false);
  points_visited[0] = true;  // Mark the initial point as visited
  inTargetRadius = false;

  // Create the kdTree
  kdtree = std::make_unique<KDTree>(points);

  // Create Annoy Tree
  for (std::size_t i = 0; i < points.size(); i++) {
    tree.add_item(i, points[i].data());
  }
  tree.build(n_trees);
  tree.save("voronoi.tree");
}

Voronoi::Voronoi(const std::size_t N, state_t x0, state_t xg, Options options)
    : tree(x0.size()), limits(options.region_limits), opt(options), xg(xg) {
  points.resize(N + 2);
  points[0] = x0;

  // Add the goal point to the points vector
  this->xg_index = 1;  // Store index for check in target_reached()
  points[xg_index] = xg;
  std::size_t state_dim = x0.size();

  // Generate N points
  if (opt.sampling_method == "random") {
    for (std::size_t i = 0; i < N; i++) points[i + 2] = this->random_state(state_dim);
  } else if (opt.sampling_method == "halton") {
    for (std::size_t i = 0; i < N; i++) points[i + 2] = this->halton_state(state_dim, i);
  } else {
    std::cerr << "Invalid sampling method" << std::endl;
    std::cerr << "Defaulting to halton sampling" << std::endl;
    for (std::size_t i = 0; i < N; i++) points[i + 2] = this->halton_state(state_dim, i);
  }

  initialize(N, x0.size());
}

Voronoi::Voronoi(const std::size_t N, const std::size_t state_dim, Options options)
    : tree(state_dim), limits(options.region_limits), opt(options) {
  points.resize(N);

  // Generate N points
  if (opt.sampling_method == "random") {
    for (std::size_t i = 0; i < N; i++) points[i] = this->random_state(state_dim);
  } else if (opt.sampling_method == "halton") {
    for (std::size_t i = 0; i < N; i++) points[i] = this->halton_state(state_dim, i);
  } else {
    std::cerr << "Invalid sampling method" << std::endl;
    std::cerr << "Defaulting to halton sampling" << std::endl;
    for (std::size_t i = 0; i < N; i++) points[i] = this->halton_state(state_dim, i);
  }

  initialize(N, state_dim);
}

Voronoi::Voronoi(Voronoi&& other) noexcept
    : tree(other.points[0].size()),
      kdtree(std::move(other.kdtree)),
      limits(std::move(other.limits)),
      opt(std::move(other.opt)),
      points(std::move(other.points)),
      points_visited(std::move(other.points_visited)),
      xg_index(other.xg_index) {
  tree.load("voronoi.tree");
}

Voronoi& Voronoi::operator=(Voronoi&& other) noexcept {
  if (this != &other) {
    kdtree = std::move(other.kdtree);
    limits = other.limits;
    opt = std::move(other.opt);
    points = std::move(other.points);
    points_visited = std::move(other.points_visited);
    xg_index = other.xg_index;
    tree.load("voronoi.tree");
  }
  return *this;
}

void Voronoi::new_query(const state_t x0, const state_t xg) {
  this->xg = xg;
  for (std::size_t i = 0; i < points.size(); i++) points_visited[i] = false;
  // Find Cell for x0
  std::vector<int> nearestIndexVec;
  tree.get_nns_by_vector(x0.data(), 1, search_k, &nearestIndexVec, nullptr);
  int x0_index = nearestIndexVec[0];
  points_visited[x0_index] = true;
  // Find Cell for xg
  nearestIndexVec.clear();
  tree.get_nns_by_vector(xg.data(), 1, search_k, &nearestIndexVec, nullptr);
  xg_index = nearestIndexVec[0];
  inTargetRadius = false;
}

bool Voronoi::visit(state_t x) {
  // Find the nearest neighbor to 'x' using the kdTree
  int nearestIndex = -1;
  // Choose the nearest neighbor based on the tree type
  if (false) {
    nearestIndex = kdtree->nearest_index(x);
  } else {
    std::vector<int> nearestIndexVec;
    tree.get_nns_by_vector(x.data(), 1, search_k, &nearestIndexVec, nullptr);
    nearestIndex = nearestIndexVec[0];
  }

  // Check if the point is already visited
  if (points_visited[nearestIndex]) {
    return false;
  }

  // Check if the point is in target radius
  if (opt.target_radius > 0.0) {
    double distance = 0.0;
    if (opt.distance_metric == nullptr) {
      // Default to Euclidean distance
      for (std::size_t i = 0; i < x.size(); i++) distance += std::pow(x[i] - this->xg[i], 2);
      distance = std::sqrt(distance);
    } else {
      // Use the provided distance metric
      distance = opt.distance_metric(this->xg, x);
    }
    this->inTargetRadius = distance <= opt.target_radius;
  }

  // Mark the nearest point as visited
  points_visited[nearestIndex] = true;
  return true;
}

std::vector<int> Voronoi::generate_primenumbers(std::size_t n) {
  // function should return the first n prime numbers
  std::vector<int> primes;
  int num = 2;
  while (primes.size() < n) {
    bool isPrime = true;
    for (int i = 2; i <= num / 2; i++) {
      if (num % i == 0) {
        isPrime = false;
        break;
      }
    }
    if (isPrime) {
      primes.push_back(num);
    }
    num++;
  }
  return primes;
}

double Voronoi::halton_sequence(int index, const int base, double lower_limit, double upper_limit) {
  // Swap the limits if the lower limit is greater than the upper limit
  if (lower_limit > upper_limit) {
    double temp = lower_limit;
    lower_limit = upper_limit;
    upper_limit = temp;
  }

  double distance = std::abs(upper_limit - lower_limit);
  double r = 0, f = distance;
  while (index > 0) {
    f = f / base;
    r = r + f * (index % base);
    index = index / base;
  }
  return r + lower_limit;
}

state_t Voronoi::halton_state(const std::size_t state_dim, const int index) {
  static std::vector<int> primes = generate_primenumbers(state_dim);
  state_t halton_state(state_dim);
  for (std::size_t i = 0; i < state_dim; i++) {
    halton_state[i] = halton_sequence(index, primes[i], limits[i].first, limits[i].second);
  }
  return halton_state;
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

bool Voronoi::target_reached() { return points_visited[xg_index] || inTargetRadius; }
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
  // Check if states is in limits
  if (!opt.state_limits.empty())
    for (auto it = begin; it < end; it += state_dim)
      for (std::size_t i = 0; i < state_dim; i++)
        if (*(it + i) < opt.state_limits[i].first || *(it + i) > opt.state_limits[i].second) return true;

  // Collision Detection Obstacles
  if (opt.dynamic_obstacles.empty() || opt.static_obstacles.empty() || begin >= end) {
    return false;
  }

  for (auto it = begin; it < end; it += state_dim) {
    for (auto& dynamicObstacle : opt.dynamic_obstacles) {
      // Transform the dynamic obstacle's polytope data based on the current state
      dynamicObstacle.polytope_data = dynamicObstacle.transform_obstacle(
          dynamicObstacle.polytope_data, dynamicObstacle.x_cur, std::vector<double>(it, it + state_dim));

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
