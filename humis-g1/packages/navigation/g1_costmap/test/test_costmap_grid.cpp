// CostmapGrid unit tests: index round-trip, world<->cell, bounds, reset.

#include <gtest/gtest.h>

#include "g1_costmap/core/costmap_grid.h"

using g1_costmap::CostmapGrid;
using g1_costmap::FREE_SPACE;
using g1_costmap::LETHAL_OBSTACLE;

TEST(CostmapGrid, IndexRoundTrip) {
  CostmapGrid g(0.1, -1.0, -2.0, 5, 4);
  ASSERT_TRUE(g.initialized());
  EXPECT_EQ(g.cells(), 20u);
  unsigned int mx = 0, my = 0;
  for (unsigned int i = 0; i < g.cells(); ++i) {
    g.indexToCells(i, mx, my);
    EXPECT_LT(mx, g.sizeX());
    EXPECT_LT(my, g.sizeY());
    EXPECT_EQ(g.index(mx, my), i);
  }
}

TEST(CostmapGrid, WorldToMapAndBack) {
  CostmapGrid g(0.5, 1.0, 2.0, 10, 10);
  unsigned int mx = 0, my = 0;
  ASSERT_TRUE(g.worldToMap(1.0, 2.0, mx, my));  // origin corner -> cell (0,0)
  EXPECT_EQ(mx, 0u);
  EXPECT_EQ(my, 0u);
  ASSERT_TRUE(g.worldToMap(1.6, 2.6, mx, my));  // +0.6/0.5 -> cell (1,1)
  EXPECT_EQ(mx, 1u);
  EXPECT_EQ(my, 1u);
  double wx = 0, wy = 0;
  g.mapToWorld(1, 1, wx, wy);  // cell center
  EXPECT_NEAR(wx, 1.0 + 1.5 * 0.5, 1e-9);
  EXPECT_NEAR(wy, 2.0 + 1.5 * 0.5, 1e-9);
}

TEST(CostmapGrid, WorldToMapRejectsOutOfBounds) {
  CostmapGrid g(0.5, 1.0, 2.0, 10, 10);  // spans [1,6) x [2,7)
  unsigned int mx = 0, my = 0;
  EXPECT_FALSE(g.worldToMap(0.5, 3.0, mx, my));  // below origin_x
  EXPECT_FALSE(g.worldToMap(3.0, 1.5, mx, my));  // below origin_y
  EXPECT_FALSE(g.worldToMap(6.1, 3.0, mx, my));  // past far x edge
  EXPECT_FALSE(g.worldToMap(3.0, 7.1, mx, my));  // past far y edge
}

TEST(CostmapGrid, SetResetReflectCosts) {
  CostmapGrid g(0.1, 0.0, 0.0, 3, 3);
  g.setCost(1, 1, LETHAL_OBSTACLE);
  EXPECT_EQ(g.at(1, 1), LETHAL_OBSTACLE);
  g.reset(FREE_SPACE);
  EXPECT_EQ(g.at(1, 1), FREE_SPACE);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
