// PlannerGrid unit tests: world<->cell round-trips, OOB, index, cell center.

#include <gtest/gtest.h>

#include "g1_global_planner/core/planner_grid.h"

using g1_global_planner::PlannerGrid;

TEST(PlannerGrid, WorldToMapAndBack) {
  // 10x10 grid, res=0.5, origin=(1.0, 2.0).
  PlannerGrid g(0.5, 1.0, 2.0, 10, 10);

  int mx = -1, my = -1;
  // Origin corner maps to cell (0,0).
  ASSERT_TRUE(g.worldToMap(1.0, 2.0, mx, my));
  EXPECT_EQ(mx, 0);
  EXPECT_EQ(my, 0);

  // +0.6 m / 0.5 res -> index 1.
  ASSERT_TRUE(g.worldToMap(1.6, 2.6, mx, my));
  EXPECT_EQ(mx, 1);
  EXPECT_EQ(my, 1);

  // Round-trip: worldToMap then mapToWorld stays within half a cell.
  for (int row = 0; row < 10; ++row) {
    for (int col = 0; col < 10; ++col) {
      double wx_in = 1.0 + (col + 0.3) * 0.5;
      double wy_in = 2.0 + (row + 0.7) * 0.5;
      int cx = -1, cy = -1;
      ASSERT_TRUE(g.worldToMap(wx_in, wy_in, cx, cy));
      double wx_out, wy_out;
      g.mapToWorld(cx, cy, wx_out, wy_out);
      EXPECT_NEAR(wx_in, wx_out, 0.5) << "col=" << col << " row=" << row;
      EXPECT_NEAR(wy_in, wy_out, 0.5) << "col=" << col << " row=" << row;
    }
  }
}

TEST(PlannerGrid, CellZeroCenterEqualsOriginPlusHalfRes) {
  PlannerGrid g(0.1, -1.0, -2.0, 5, 4);
  double wx, wy;
  g.mapToWorld(0, 0, wx, wy);
  EXPECT_NEAR(wx, -1.0 + 0.5 * 0.1, 1e-9);
  EXPECT_NEAR(wy, -2.0 + 0.5 * 0.1, 1e-9);
}

TEST(PlannerGrid, OutOfBoundsReturnsFalse) {
  // Grid spans [1.0, 6.0) x [2.0, 7.0) in world coords.
  PlannerGrid g(0.5, 1.0, 2.0, 10, 10);
  int mx, my;
  EXPECT_FALSE(g.worldToMap(0.5, 3.0, mx, my));  // below origin_x
  EXPECT_FALSE(g.worldToMap(3.0, 1.5, mx, my));  // below origin_y
  EXPECT_FALSE(g.worldToMap(6.1, 3.0, mx, my));  // past far x
  EXPECT_FALSE(g.worldToMap(3.0, 7.1, mx, my));  // past far y
}

TEST(PlannerGrid, KnownIndexMapsCorrectly) {
  // 3x4 grid: index = my*3 + mx. Cell (2,3) -> index 11.
  PlannerGrid g(0.1, 0.0, 0.0, 3, 4);
  // Verify that at(2,3) accesses the right index by writing via setAt.
  g.setAt(2, 3, static_cast<int8_t>(42));
  EXPECT_EQ(g.data()[3 * 3 + 2], static_cast<int8_t>(42));
  EXPECT_EQ(g.at(2, 3), static_cast<int8_t>(42));
}

TEST(PlannerGrid, SizedConstructorFillsDefault) {
  PlannerGrid g(0.1, 0.0, 0.0, 5, 5, static_cast<int8_t>(-1));
  EXPECT_EQ(g.cells(), 25);
  for (int y = 0; y < 5; ++y)
    for (int x = 0; x < 5; ++x)
      EXPECT_EQ(g.at(x, y), static_cast<int8_t>(-1));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
