#include "graph.h"

Graph::Graph(std::shared_ptr<const cspace::state_t> x0_ptr) : success_flag(false) { add_vertex(x0_ptr); }

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

cspace::input_trajectory_t Graph::get_input(std::shared_ptr<const cspace::state_t> x) {
  cspace::input_trajectory_t path;
  Vertex* v = vertex_map.at(x);

  // While not origin vertex
  while (v->edge_in != nullptr) {
    auto u_ptr = v->edge_in->input;
    path.insert(path.end(), u_ptr->begin(), u_ptr->end());
    v = v->edge_in->source;
  }
  return path;
}

auto Graph::begin() -> decltype(vertices.begin()) { return vertices.begin(); }
auto Graph::end() -> decltype(vertices.end()) { return vertices.end(); }
auto Graph::cbegin() -> decltype(vertices.cbegin()) { return vertices.cbegin(); }
auto Graph::cend() -> decltype(vertices.cend()) { return vertices.cend(); }
