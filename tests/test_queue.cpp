#include "queue.h"
#include "gtest/gtest.h"

namespace cs = cspace;

TEST(Queue, push){
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{0, 0});
  auto x1_ptr = std::make_shared<cs::state_t>(cs::state_t{1, 1});
  auto x2_ptr = std::make_shared<cs::state_t>(cs::state_t{-1, -2});
  cs::input_trajectory_t u = {{1, 1}, {0, 0}, {0,1}, {1, 0}};
  auto u_ptr = std::make_shared<cs::input_trajectory_t>(u);

  Queue Q(x0_ptr);
  auto [ptr,cost] = Q.pop();
  Q.push(x1_ptr, cost, u_ptr, 2);
  auto [ptr2,cost2] = Q.pop();
  EXPECT_EQ(*ptr, *x0_ptr);
  EXPECT_EQ(ptr, x0_ptr);
  EXPECT_EQ(cost2, 2);
}

TEST(Queue, pop){
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{0, 0});
  auto x1_ptr = std::make_shared<cs::state_t>(cs::state_t{1, 1});
  auto x2_ptr = std::make_shared<cs::state_t>(cs::state_t{-1, -2});
  cs::input_trajectory_t u = {{1, 1}, {0, 0}, {0,1}, {1, 0}};
  auto u_ptr = std::make_shared<cs::input_trajectory_t>(u);

  Queue Q(x0_ptr);
  auto [ptr,cost] = Q.pop();
  EXPECT_EQ(*ptr, *x0_ptr);
  EXPECT_EQ(ptr, x0_ptr);

}
