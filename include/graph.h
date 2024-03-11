#ifndef GRAPH_H
#define GRAPH_H
#include <cwchar>
#include <list>
#include <unordered_map>
#include <vector>

#include "cspace.h"

class Graph {
 public:
  struct Vertex;
  struct Edge;

 private:
  std::unordered_map<std::shared_ptr<const cspace::state_t>, Vertex*> vertex_map;
  bool success_flag;
  std::list<Vertex> vertices;
  std::list<Edge> edges;

 public:
  Graph(std::shared_ptr<const cspace::state_t> x0_ptr);
  Graph(const Graph& other);
  Graph(Graph&& other);
  Graph& operator=(const Graph& other);
  Graph& operator=(Graph&& other);
  void add_vertex(std::shared_ptr<const cspace::state_t> x);
  void add_edge(std::shared_ptr<const cspace::state_t> x1, std::shared_ptr<const cspace::state_t> x2,
                cspace::input_traj_ptr u, float time);
  std::pair<cspace::input_trajectory_t,float> get_input(std::shared_ptr<const cspace::state_t> x);
  cspace::trajectory_t get_trajectory(std::shared_ptr<const cspace::state_t> x);
  Vertex& front();
  Vertex& back();
  auto begin() -> decltype(vertices.begin());
  auto end() -> decltype(vertices.end());
  auto cbegin() -> decltype(vertices.cbegin());
  auto cend() -> decltype(vertices.cend());

 public:
  bool get_success();
  void set_success(bool flag);
  std::size_t size_vertices();
};

struct Graph::Vertex {
  std::shared_ptr<const cspace::state_t> state;
  std::list<Edge*> edges_out;
  Edge* edge_in = nullptr;
  Vertex(std::shared_ptr<const cspace::state_t> x) : state(x) {}
};

struct Graph::Edge {
  Vertex* source;
  Vertex* target;
  std::shared_ptr<const cspace::input_trajectory_t> input;
  float time;
  Edge(Vertex* s, Vertex* t, std::shared_ptr<const cspace::input_trajectory_t> u, float time)
      : source(s), target(t), input(u), time(time) {}
};

#endif  // GRAPH_H
