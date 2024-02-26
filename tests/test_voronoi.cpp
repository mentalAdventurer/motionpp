#include <iostream>

#include "cspace.h"
#include "gtest/gtest.h"

using namespace cspace;
using iterVec = std::vector<double>::const_iterator;

namespace cs = cspace;

TEST(Voronoi, constructor){
  state_t x0 = {0, 0, 0, 0};
  state_t xg = {1, 0, 0, 0};
  Voronoi P(10000, x0, xg);
}

TEST(Voronoi, visit){
  state_t x0 = {0, 0, 0, 0};
  state_t xg = {1, 0, 0, 0};
  Voronoi P(10000, x0, xg);
  state_t x = {0.5, 0, 0, 0};
  EXPECT_TRUE(P.visit(x));
  EXPECT_FALSE(P.visit(x));
}

TEST(Voronoi,target_reached){
  state_t x0 = {0, 0, 0, 0};
  state_t xg = {1, 0, 0, 0};
  Voronoi P(10000, x0, xg);
  EXPECT_FALSE(P.target_reached()) << "Voronoi just initialised. Target should not be reached yet";
  state_t x = {10, 2, 7, 4};
  P.visit(x);
  EXPECT_FALSE(P.target_reached()) << "Target should not be reached yet";
  P.visit(xg);
  EXPECT_TRUE(P.target_reached()) << "Target should be reached now";
}
