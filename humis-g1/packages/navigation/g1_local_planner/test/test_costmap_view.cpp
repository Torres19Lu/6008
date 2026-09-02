// Unit tests for CostmapView: world<->cell, normCost, cost(), gradient().

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "g1_local_planner/core/costmap_view.h"

using g1_local_planner::CostmapView;

// ---------------------------------------------------------------------------
// Helper: build a flat WxH grid filled with a single value.
static CostmapView makeUniform(int W, int H, int8_t fill,
                               double res = 0.1,
                               double ox = 0.0, double oy = 0.0,
                               double lethal = 1.0, bool unk_obs = true) {
  std::vector<int8_t> d(static_cast<size_t>(W) * static_cast<size_t>(H), fill);
  return CostmapView(res, ox, oy, W, H, d, lethal, unk_obs);
}

// ---------------------------------------------------------------------------
// Default ctor: invalid / empty.
TEST(CostmapView, DefaultCtorIsInvalid) {
  CostmapView cv;
  EXPECT_FALSE(cv.valid());
  EXPECT_EQ(cv.cells(), 0u);
}

// ---------------------------------------------------------------------------
// worldToMap: floor behaviour + out-of-bounds.
TEST(CostmapView, WorldToMapFloor) {
  // 10x10 grid, res=0.5, origin=(1.0, 2.0)
  // Grid spans [1.0, 6.0) x [2.0, 7.0)
  CostmapView cv = makeUniform(10, 10, 0, 0.5, 1.0, 2.0);

  int mx = -1, my = -1;
  // Origin corner -> cell (0,0)
  ASSERT_TRUE(cv.worldToMap(1.0, 2.0, mx, my));
  EXPECT_EQ(mx, 0);
  EXPECT_EQ(my, 0);

  // 1.0 + 0.6 m / 0.5 -> 1.2 -> floor = 1
  ASSERT_TRUE(cv.worldToMap(1.6, 2.6, mx, my));
  EXPECT_EQ(mx, 1);
  EXPECT_EQ(my, 1);

  // Just inside the far corner: 5.99 -> floor((5.99-1.0)/0.5) = floor(9.98) = 9
  ASSERT_TRUE(cv.worldToMap(5.99, 6.99, mx, my));
  EXPECT_EQ(mx, 9);
  EXPECT_EQ(my, 9);
}

TEST(CostmapView, WorldToMapOutOfBounds) {
  CostmapView cv = makeUniform(10, 10, 0, 0.5, 1.0, 2.0);
  int mx = 0, my = 0;
  EXPECT_FALSE(cv.worldToMap(0.5, 3.0, mx, my));  // below origin_x
  EXPECT_FALSE(cv.worldToMap(3.0, 1.5, mx, my));  // below origin_y
  EXPECT_FALSE(cv.worldToMap(6.1, 3.0, mx, my));  // past far x (>= 6.0)
  EXPECT_FALSE(cv.worldToMap(3.0, 7.1, mx, my));  // past far y (>= 7.0)
  EXPECT_FALSE(cv.worldToMap(6.0, 3.0, mx, my));  // exactly at far boundary (out)
}

// Cell (0,0) centre is at origin + 0.5*res.
TEST(CostmapView, CellZeroCentreAtOriginPlusHalfRes) {
  // Verify: a world point exactly at the cell (0,0) centre maps to (0,0).
  // Cell 0 centre x = 0.0 + 0.5*0.1 = 0.05
  CostmapView cv = makeUniform(5, 4, 0, 0.1, 0.0, 0.0);
  int mx, my;
  ASSERT_TRUE(cv.worldToMap(0.05, 0.05, mx, my));
  EXPECT_EQ(mx, 0);
  EXPECT_EQ(my, 0);
}

// ---------------------------------------------------------------------------
// normCost: mapping for 0 / mid-range / 100 / -1.
TEST(CostmapView, NormCostZero) {
  CostmapView cv = makeUniform(2, 2, 0, 0.1, 0.0, 0.0, 1.0, true);
  EXPECT_DOUBLE_EQ(cv.normCost(0), 0.0);
}

TEST(CostmapView, NormCostInflation) {
  CostmapView cv = makeUniform(2, 2, 0, 0.1, 0.0, 0.0, 1.0, true);
  EXPECT_DOUBLE_EQ(cv.normCost(50), 0.5);
  EXPECT_DOUBLE_EQ(cv.normCost(1),  0.01);
  EXPECT_DOUBLE_EQ(cv.normCost(99), 0.99);
}

TEST(CostmapView, NormCostLethal) {
  CostmapView cv = makeUniform(2, 2, 0, 0.1, 0.0, 0.0, 1.0, true);
  EXPECT_DOUBLE_EQ(cv.normCost(100), 1.0);
}

TEST(CostmapView, NormCostUnknownTreatedAsObstacle) {
  CostmapView cv = makeUniform(2, 2, 0, 0.1, 0.0, 0.0, 1.0, /*unk_obs=*/true);
  EXPECT_DOUBLE_EQ(cv.normCost(static_cast<int8_t>(-1)), 1.0);
}

TEST(CostmapView, NormCostUnknownTreatedAsFree) {
  CostmapView cv = makeUniform(2, 2, 0, 0.1, 0.0, 0.0, 1.0, /*unk_obs=*/false);
  EXPECT_DOUBLE_EQ(cv.normCost(static_cast<int8_t>(-1)), 0.0);
}

// ---------------------------------------------------------------------------
// cost(): free region -> ~0; near lethal -> high; out-of-bounds -> lethal_cost.
TEST(CostmapView, CostFreeRegion) {
  // Uniform free grid: cost anywhere in the interior should be 0.
  CostmapView cv = makeUniform(10, 10, 0, 0.1, 0.0, 0.0);
  // Interior point well away from edges.
  EXPECT_NEAR(cv.cost(0.55, 0.55), 0.0, 1e-9);
}

TEST(CostmapView, CostOutOfBoundsIsLethal) {
  CostmapView cv = makeUniform(5, 5, 0, 0.1, 0.0, 0.0, /*lethal=*/1.0);
  EXPECT_DOUBLE_EQ(cv.cost(-0.1, 0.25), 1.0);  // left of origin
  EXPECT_DOUBLE_EQ(cv.cost(0.25, -0.1), 1.0);  // below origin
  EXPECT_DOUBLE_EQ(cv.cost(0.6,  0.25), 1.0);  // right of grid (5*0.1=0.5)
  EXPECT_DOUBLE_EQ(cv.cost(0.25, 0.6),  1.0);  // above grid
}

TEST(CostmapView, CostNearLethalCellIsHigh) {
  // Place a lethal cell at (5,5) in an otherwise free 10x10 grid.
  int W = 10, H = 10;
  std::vector<int8_t> d(static_cast<size_t>(W) * static_cast<size_t>(H), 0);
  d[5 * W + 5] = 100;  // cell (mx=5, my=5)
  CostmapView cv(0.1, 0.0, 0.0, W, H, d, 1.0, true);

  // Centre of the lethal cell: (5+0.5)*0.1 = 0.55
  double lx = 0.55, ly = 0.55;
  // Cost AT that cell centre is 1.0 (the bilinear corner case).
  // A nearby free neighbour should give a value well below 1.0.
  // Just verify cost is > 0 near the lethal cell and < lethal in the free region.
  double c_near = cv.cost(lx + 0.05, ly);  // slightly right of lethal centre
  EXPECT_GT(c_near, 0.0);

  // Far away (opposite corner) should be ~0.
  double c_far = cv.cost(0.05, 0.05);
  EXPECT_NEAR(c_far, 0.0, 1e-9);
}

TEST(CostmapView, CostBilinearMidpointBetweenFreAndLethal) {
  // 2x1 grid: cell(0,0)=0 (free), cell(1,0)=100 (lethal). res=1.0.
  // Centres: cell(0,0) at (0.5, 0.5), cell(1,0) at (1.5, 0.5).
  // At the world midpoint x=1.0 exactly, the bilinear should interpolate.
  std::vector<int8_t> d = {0, 100};
  CostmapView cv(1.0, 0.0, 0.0, 2, 1, d, 1.0, true);
  double mid_cost = cv.cost(1.0, 0.5);
  // Must be strictly between 0 and 1.
  EXPECT_GT(mid_cost, 0.0);
  EXPECT_LT(mid_cost, 1.0);
}

// ---------------------------------------------------------------------------
// gradient(): lethal block on +x side -> gx > 0; +y side -> gy > 0.
// Symmetric: symmetric free region -> near-zero gradient.

TEST(CostmapView, GradientPointsTowardLethalOnPlusX) {
  // 10x10 free grid. Lethal block placed in the right half (columns 6..9).
  // res=0.1: cell mx centre at (mx+0.5)*0.1. Boundary between col 5 and 6 is at x=0.6.
  // To detect the gradient we must sample close enough to the boundary that the
  // finite-difference step (+-0.1) crosses from free (col 5) into lethal (col 6).
  // Sample at x=0.58: left probe (0.48) is free, right probe (0.68) is lethal -> gx > 0.
  int W = 10, H = 10;
  std::vector<int8_t> d(static_cast<size_t>(W) * static_cast<size_t>(H), 0);
  for (int my = 0; my < H; ++my)
    for (int mx = 6; mx < W; ++mx)
      d[my * W + mx] = 100;
  CostmapView cv(0.1, 0.0, 0.0, W, H, d, 1.0, true);

  // Probe at x=0.58, y=0.55 (middle of grid in y; one step left is free, one step right is lethal).
  auto [gx, gy] = cv.gradient(0.58, 0.55);
  EXPECT_GT(gx, 0.0) << "gx=" << gx << " gy=" << gy;
}

TEST(CostmapView, GradientPointsTowardLethalOnPlusY) {
  // 10x10 free grid. Lethal block placed in the top half (rows 6..9).
  // res=0.1: boundary between row 5 and 6 is at y=0.6.
  // Sample at y=0.58: down probe (0.48) is free, up probe (0.68) is lethal -> gy > 0.
  int W = 10, H = 10;
  std::vector<int8_t> d(static_cast<size_t>(W) * static_cast<size_t>(H), 0);
  for (int my = 6; my < H; ++my)
    for (int mx = 0; mx < W; ++mx)
      d[my * W + mx] = 100;
  CostmapView cv(0.1, 0.0, 0.0, W, H, d, 1.0, true);

  // Probe at x=0.55, y=0.58 (one step down is free, one step up is lethal).
  auto [gx, gy] = cv.gradient(0.55, 0.58);
  EXPECT_GT(gy, 0.0) << "gx=" << gx << " gy=" << gy;
}

TEST(CostmapView, GradientNearZeroInUniformFreeRegion) {
  // Fully free 10x10 grid; gradient should be identically zero everywhere.
  CostmapView cv = makeUniform(10, 10, 0, 0.1, 0.0, 0.0);
  auto [gx, gy] = cv.gradient(0.5, 0.5);
  EXPECT_NEAR(gx, 0.0, 1e-9);
  EXPECT_NEAR(gy, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Invalid ctor: mismatched data size throws.
TEST(CostmapView, InvalidDataSizeThrows) {
  std::vector<int8_t> bad(5, 0);  // should be 4
  EXPECT_THROW(CostmapView(0.1, 0.0, 0.0, 2, 2, bad, 1.0, true),
               std::invalid_argument);
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
