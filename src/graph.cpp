#include "graph.h"

Graph::Graph(std::shared_ptr<const cspace::state_t> x0_ptr) : success_flag(false) { add_vertex(x0_ptr); }

Graph::Graph(const Graph& other) : success_flag(other.success_flag) {
  for (auto& v : other.vertices) {
    add_vertex(v.state);
  }
  for (auto& e : other.edges) {
    add_edge(e.source->state, e.target->state, e.input, e.time);
  }
}

Graph::Graph(Graph&& other)
    : vertex_map(std::move(other.vertex_map)),
      success_flag(other.success_flag),
      vertices(std::move(other.vertices)),
      edges(std::move(other.edges)) {}

Graph& Graph::operator=(const Graph& other) {
  success_flag = other.success_flag;
  for (auto& v : other.vertices) {
    add_vertex(v.state);
  }
  for (auto& e : other.edges) {
    add_edge(e.source->state, e.target->state, e.input, e.time);
  }
  return *this;
}

Graph& Graph::operator=(Graph&& other) {
  if (this != &other) {
    vertex_map = std::move(other.vertex_map);
    success_flag = other.success_flag;
    vertices = std::move(other.vertices);
    edges = std::move(other.edges);
  }
  return *this;
}

void Graph::add_vertex(std::shared_ptr<const cspace::state_t> x) {
  vertices.emplace_back(x);
  vertex_map[x] = &vertices.back();
}

void Graph::add_edge(std::shared_ptr<const cspace::state_t> x1, std::shared_ptr<const cspace::state_t> x2,
                     cspace::input_traj_ptr u, float time) {
  Vertex* v1 = vertex_map.at(x1);
  Vertex* v2 = vertex_map.at(x2);
  edges.emplace_back(v1, v2, u, time);
  v1->edges_out.push_back(&edges.back());
  v2->edge_in = &edges.back();
}

bool Graph::get_success() { return success_flag; }

void Graph::set_success(bool flag) { success_flag = flag; }

std::size_t Graph::size_vertices() { return vertices.size(); }

std::pair<cspace::input_trajectory_t, float> Graph::get_input(std::shared_ptr<const cspace::state_t> x) {
  cspace::input_trajectory_t path;
  Vertex* v = vertex_map.at(x);
  float time = 0;

  // While not origin vertex
  while (v->edge_in != nullptr) {
    auto u_ptr = v->edge_in->input;
    time += v->edge_in->time;
    path.insert(path.end(), u_ptr->rbegin(), u_ptr->rend());
    v = v->edge_in->source;
  }
  std::reverse(path.begin(), path.end());
  return {path, time};
}

cspace::trajectory_t Graph::get_trajectory(std::shared_ptr<const cspace::state_t> x) {
  cspace::trajectory_t path;
  Vertex* v = vertex_map.at(x);
  path.emplace_back(*(v->state));

  // While not origin vertex
  while (v->edge_in != nullptr) {
    v = v->edge_in->source;
    path.emplace_back(*(v->state));
  }
  std::reverse(path.begin(), path.end());
  return path;
}

Graph::Vertex& Graph::front() { return vertices.front(); }
Graph::Vertex& Graph::back() { return vertices.back(); }
auto Graph::begin() -> decltype(vertices.begin()) { return vertices.begin(); }
auto Graph::end() -> decltype(vertices.end()) { return vertices.end(); }
auto Graph::cbegin() -> decltype(vertices.cbegin()) { return vertices.cbegin(); }
auto Graph::cend() -> decltype(vertices.cend()) { return vertices.cend(); }
