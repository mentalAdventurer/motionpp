#include <iostream>

#include "cspace.h"
#include "gtest/gtest.h"

using namespace cspace;
using iterVec = std::vector<double>::const_iterator;

namespace cs = cspace;

std::vector<double> eulerIntegrate(std::vector<double>::const_iterator x0,
                                               std::vector<double>::const_iterator u_traj,
                                               std::size_t state_dim, std::size_t traj_dim, float dt);
std::pair<std::vector<double>, std::vector<float>> getMotionPrimitives(const cs::state_t &x0);

TEST(ReachedSet, constructor) {
  ReachedSet R(eulerIntegrate, getMotionPrimitives);
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{0, 0, 0, 0});
  R(x0_ptr);
}

TEST(ReachedSet, bracket_operator) {
  ReachedSet R(eulerIntegrate, getMotionPrimitives);
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{-1, -2,4,-123123123});
  R(x0_ptr);
}

TEST(ReachedSet, empty) {
  ReachedSet R(eulerIntegrate, getMotionPrimitives);
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{0, 0,0,0});
  auto x1_ptr = std::make_shared<cs::state_t>(cs::state_t{1, 1,1.123123,10000});
  auto x2_ptr = std::make_shared<cs::state_t>(cs::state_t{-1, -2,4,-123123123});
  EXPECT_TRUE(R.empty());
  R(x0_ptr);
  R(x1_ptr);
  R(x2_ptr);
  EXPECT_FALSE(R.empty());
  R.clear();
  EXPECT_TRUE(R.empty());
}

TEST(ReachedSet, pop_input_ptr) {
  ReachedSet R(eulerIntegrate, getMotionPrimitives);
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{0, 0, 0, 0});
  R(x0_ptr);
  auto [input, time] = R.pop_input_ptr();
  EXPECT_EQ(1, input->size());
  EXPECT_FLOAT_EQ(0.2, time) << "time: " << time;
  EXPECT_FLOAT_EQ(1.0, input->at(0).at(0)) << "input: " << input->at(0).at(0);
}

TEST(ReachedSet, pop_state_ptr) {
  ReachedSet R(eulerIntegrate, getMotionPrimitives);
  auto x0_ptr = std::make_shared<cs::state_t>(cs::state_t{0, 0, 0, 0});
  R(x0_ptr);
  state_ptr x_next_ptr = R.pop_state_ptr();
  EXPECT_EQ(4, x_next_ptr->size());
}

using vec_iter = std::vector<double>::const_iterator;
std::vector<double> dynamics(vec_iter x, vec_iter u, std::size_t n) {
  assert (n == 4); 
  const float d = 0.1;
  const float k = 1;
  const float m = 1;

  cs::state_t dxdt(n);
  dxdt[0] = x[1];
  dxdt[1] = (-d * x[1] - k * x[0] + u[0]) / m;
  dxdt[2] = x[3];
  dxdt[3] = dxdt[1] - u[0] / m;

  return dxdt;
}

std::vector<double> eulerIntegrate(std::vector<double>::const_iterator x0,
                                               std::vector<double>::const_iterator u_traj,
                                               std::size_t state_dim, std::size_t traj_dim, float dt) {
  std::vector<double> x_traj(traj_dim * state_dim + state_dim);
  std::copy(x0, x0 + state_dim, x_traj.begin());

  for (std::size_t j = 0; j < traj_dim; j++) {
    auto dxdt = dynamics(x_traj.begin() + j * state_dim, u_traj + j * state_dim, state_dim);
    for (std::size_t k = 0; k < state_dim; k++) {
      x_traj[j * state_dim + k + state_dim] = x_traj[j * state_dim + k] + dxdt[k] * dt;
    }
  }
  return x_traj;
}


std::pair<std::vector<double>, std::vector<float>> getMotionPrimitives(const cs::state_t &x0) {
  std::vector<double> motion_primitives = {1.0, 2.0, 4.0, 2.7, 1.0, 2.0, 4.0, 2.7};
  std::vector<float> times = {0.1, 0.2};
  return std::make_pair(motion_primitives, times);
}
