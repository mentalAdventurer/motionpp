#include "graph.h"
#include "gtest/gtest.h"

namespace cs = cspace;

TEST(Graph, addEdge) {
  cs::state_t x0 = {0, 0};
  cs::state_t x1 = {1, 1};
  cs::state_t x2 = {-1, -2};
  cs::input_trajectory_t u = {{1, 0}, {0, 1}};
  Graph G(x0);
  G.add_vertex(x1);
  G.add_edge(x0, x1, u);
  G.add_vertex(x2);
  G.add_edge(x1, x2, u);
  G.add_edge(x0, x2, u);
}

TEST(Graph, addVertex) {
  cs::state_t x0 = {0, 0};
  Graph G(x0);
  G.add_vertex({0, 0});
  G.add_vertex({1, 1});
}

TEST(Graph, getInput) {
  cs::state_t x0 = {0, 0};
  cs::state_t x1 = {1, 1};
  cs::state_t x2 = {-1, -2};
  cs::input_trajectory_t u = {{1, 0}, {0, 1}, {1, 1}};
  Graph G(x0);
  G.add_vertex(x1);
  G.add_edge(x0, x1, u);
  G.add_vertex(x2);
  G.add_edge(x1, x2, u);
  auto path = G.get_input(x2);
  std::list<cs::input_t> expected = {{1, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 1}, {1, 1}};

  EXPECT_EQ(path.size(), 6);
  if (!std::equal(expected.begin(), expected.end(), path.begin(), path.end())) {
    FAIL() << "Path is not as expected";
  }
}
