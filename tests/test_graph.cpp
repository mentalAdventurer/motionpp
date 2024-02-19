#include "graph.h"
#include "gtest/gtest.h"

namespace cs = cspace;

TEST(Graph, addEdge) {
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{0, 0});
  auto x1_ptr = std::make_shared<cs::state_t>(cs::state_t{1, 1});
  auto x2_ptr = std::make_shared<cs::state_t>(cs::state_t{-1, -2});
  cs::input_trajectory_t u = {{1, 0}, {0, 1}};
  auto u_ptr = std::make_shared<cs::input_trajectory_t>(u);

  Graph G(x0_ptr);
  G.add_vertex(x1_ptr);
  G.add_vertex(x2_ptr);
  G.add_edge(x1_ptr, x2_ptr, u_ptr);
  G.add_edge(x1_ptr, x2_ptr, u_ptr);
  G.add_edge(x0_ptr , x2_ptr, u_ptr);
}

TEST(Graph, addVertex) {
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{0, 0});
  auto x1_ptr = std::make_shared<cs::state_t>(cs::state_t{1, 1});
  Graph G(x0_ptr);
  G.add_vertex(x0_ptr);
  G.add_vertex(x1_ptr);
}

TEST(Graph, getInput) {
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{0, 0});
  auto x1_ptr = std::make_shared<cs::state_t>(cs::state_t{1, 1});
  auto x2_ptr = std::make_shared<cs::state_t>(cs::state_t{-1, -2});
  cs::input_trajectory_t u = {{1, 1}, {0, 0}, {0,1}, {1, 0}};
  auto u_ptr = std::make_shared<cs::input_trajectory_t>(u);
  Graph G(x0_ptr);
  G.add_vertex(x1_ptr);
  G.add_edge(x0_ptr, x1_ptr, u_ptr);
  G.add_vertex(x2_ptr);
  G.add_edge(x1_ptr, x2_ptr, u_ptr);
  auto path = G.get_input(x2_ptr);
  std::list<cs::input_t> expected = {{1, 1}, {0, 0}, {0,1}, {1, 0}, {1, 1}, {0, 0}, {0,1}, {1, 0}};
  EXPECT_EQ(path.size(), 8);
  if (!std::equal(expected.begin(), expected.end(), path.begin(), path.end())) {
    FAIL() << "Path is not as expected";
  }
}
