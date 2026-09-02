// Path smoother unit tests: staircase collapses, obstacle mid-point, flags, endpoints.

#include <gtest/gtest.h>

#include "g1_global_planner/core/cost_model.h"
#include "g1_global_planner/core/plan_types.h"
#include "g1_global_planner/core/planner_grid.h"
#include "g1_global_planner/planner/path_smoother.h"

using g1_global_planner::Cell;
using g1_global_planner::CostModel;
using g1_global_planner::PlannerConfig;
using g1_global_planner::PlannerGrid;

// 10x10 open grid at res=1.0.
static PlannerGrid makeOpen10() {
  return PlannerGrid(1.0, 0.0, 0.0, 10, 10, static_cast<int8_t>(0));
}

TEST(PathSmoother, StaircaseCollapsesToTwoCells) {
  // Manhattan staircase from (0,0) to (5,5): go right then up alternately.
  // On an open grid, the direct diagonal is clear so it collapses to {(0,0),(5,5)}.
  PlannerGrid g = makeOpen10();
  PlannerConfig cfg;
  cfg.smooth_enable = true;
  CostModel cm(cfg);

  std::vector<Cell> stair;
  for (int i = 0; i <= 5; ++i) stair.push_back({i, i});  // diagonal staircase

  auto result = smoothPath(g, cm, cfg, stair);
  // Direct line of sight (0,0)->(5,5) on open grid -> collapses to two cells.
  ASSERT_GE(static_cast<int>(result.size()), 2);
  EXPECT_EQ(result.front(), (Cell{0, 0}));
  EXPECT_EQ(result.back(),  (Cell{5, 5}));
  // With a fully clear grid the smoother should find the direct connection.
  EXPECT_EQ(static_cast<int>(result.size()), 2);
}

TEST(PathSmoother, ObstacleForcesWaypoint) {
  // Open 10x10 but block a cell at (2,2) that the direct line (0,0)->(5,5) passes.
  PlannerGrid g = makeOpen10();
  g.setAt(2, 2, static_cast<int8_t>(100));
  PlannerConfig cfg;
  cfg.smooth_enable = true;
  CostModel cm(cfg);

  // lineOfSight from (0,0) to (5,5) passes through (2,2) - should be false.
  EXPECT_FALSE(lineOfSight(g, cm, {0, 0}, {5, 5}));

  // Diagonal staircase.
  std::vector<Cell> stair;
  for (int i = 0; i <= 5; ++i) stair.push_back({i, i});

  auto result = smoothPath(g, cm, cfg, stair);
  // Cannot collapse fully - must keep an intermediate waypoint.
  EXPECT_GT(static_cast<int>(result.size()), 2);
  EXPECT_EQ(result.front(), (Cell{0, 0}));
  EXPECT_EQ(result.back(),  (Cell{5, 5}));
}

TEST(PathSmoother, SmoothDisabledReturnsInput) {
  PlannerGrid g = makeOpen10();
  PlannerConfig cfg;
  cfg.smooth_enable = false;
  CostModel cm(cfg);

  std::vector<Cell> input = {{0,0},{1,0},{2,0},{3,0}};
  auto result = smoothPath(g, cm, cfg, input);
  EXPECT_EQ(result, input);
}

TEST(PathSmoother, TwoPointPathReturnedUnchanged) {
  PlannerGrid g = makeOpen10();
  PlannerConfig cfg;
  cfg.smooth_enable = true;
  CostModel cm(cfg);

  std::vector<Cell> two = {{0,0},{5,5}};
  auto result = smoothPath(g, cm, cfg, two);
  EXPECT_EQ(result, two);
}

TEST(PathSmoother, FirstAndLastAlwaysPreserved) {
  PlannerGrid g = makeOpen10();
  g.setAt(3, 3, static_cast<int8_t>(100));
  PlannerConfig cfg;
  cfg.smooth_enable = true;
  CostModel cm(cfg);

  std::vector<Cell> path;
  for (int i = 0; i <= 7; ++i) path.push_back({i, i});

  auto result = smoothPath(g, cm, cfg, path);
  EXPECT_EQ(result.front(), path.front());
  EXPECT_EQ(result.back(),  path.back());
}

// ---- Finding 2: lineOfSight failure-mode tests for non-diagonal segments ----

// Helper: 5x5 open grid at res=1.0.
static PlannerGrid makeOpen5() {
  return PlannerGrid(1.0, 0.0, 0.0, 5, 5, static_cast<int8_t>(0));
}

// (0,2)->(4,0): asymmetric segment dx=4, dy=-2.
// Hand-verification (cell centers at (cx+0.5, cy+0.5)):
//   start=(0.5,2.5), end=(4.5,0.5). Slope dy/dx = -0.5.
//   Supercover: (0,2),(1,2),(1,1),(2,1),(3,1),(3,0),(4,0).
//   (2,1) IS on the path; (2,0) is NOT on the path.
TEST(PathSmoother, LineOfSight_Asymmetric_4_2_BlockedOnPath) {
  PlannerGrid g = makeOpen5();
  PlannerConfig cfg;
  CostModel cm(cfg);

  // (2,1) is on the true supercover of (0,2)->(4,0) -> must return false.
  g.setAt(2, 1, static_cast<int8_t>(100));
  EXPECT_FALSE(lineOfSight(g, cm, {0, 2}, {4, 0}))
      << "Segment (0,2)->(4,0) passes through (2,1); must return false when (2,1) is lethal";
}

TEST(PathSmoother, LineOfSight_Asymmetric_4_2_FreeReturnsTrue) {
  PlannerGrid g = makeOpen5();
  PlannerConfig cfg;
  CostModel cm(cfg);

  // Same segment, all cells free -> must return true.
  EXPECT_TRUE(lineOfSight(g, cm, {0, 2}, {4, 0}));
}

// (0,2)->(4,0): cell NOT on supercover should not block.
// (2,0) is NOT on the supercover; blocking it alone must not make LOS return false.
TEST(PathSmoother, LineOfSight_Asymmetric_4_2_OffPathCellDoesNotBlock) {
  PlannerGrid g = makeOpen5();
  PlannerConfig cfg;
  CostModel cm(cfg);

  g.setAt(2, 0, static_cast<int8_t>(100));  // off the supercover
  EXPECT_TRUE(lineOfSight(g, cm, {0, 2}, {4, 0}))
      << "Cell (2,0) is not on the supercover of (0,2)->(4,0); must return true";
}

// (0,1)->(3,0): dx=3, dy=-1.
// Integer-DDA: tDeltaX=2*1=2, tDeltaY=2*3=6, tMaxX=1, tMaxY=3.
// Step 1: tMaxX(1)<tMaxY(3): x=1, tMaxX=3. Check (1,1).
// Step 2: tMaxX(3)==tMaxY(3): corner! Check (2,1) and (1,0). Advance to (2,0), tMaxX=5, tMaxY=9.
// Step 3: tMaxX(5)<tMaxY(9): x=3. Check (3,0)=b. Done.
// Supercover: (0,1),(1,1),(2,0),(3,0). Corner check also covers (2,1) and (1,0).
// (2,0) IS on the main traversal path.
TEST(PathSmoother, LineOfSight_Asymmetric_3_1_BlockedAtCell20) {
  PlannerGrid g = makeOpen5();
  PlannerConfig cfg;
  CostModel cm(cfg);

  // (2,0) is on the supercover of (0,1)->(3,0).
  g.setAt(2, 0, static_cast<int8_t>(100));
  EXPECT_FALSE(lineOfSight(g, cm, {0, 1}, {3, 0}))
      << "Segment (0,1)->(3,0) passes through (2,0); must return false when (2,0) is lethal";
}

// (0,3)->(3,0): dx=3, dy=-3. Pure anti-diagonal.
// tDeltaX=6, tDeltaY=6, tMaxX=3, tMaxY=3. Every step is a corner crossing.
// Corner sequence: from (0,3) corner-check (1,3)+(0,2), advance to (1,2);
//   from (1,2) corner-check (2,2)+(1,1), advance to (2,1);
//   from (2,1) corner-check (3,1)+(2,0), advance to (3,0)=b.
// (1,2) is on the main diagonal path.
TEST(PathSmoother, LineOfSight_AntiDiagonal_3_3_BlockedOnPath) {
  PlannerGrid g = makeOpen5();
  PlannerConfig cfg;
  CostModel cm(cfg);

  g.setAt(1, 2, static_cast<int8_t>(100));
  EXPECT_FALSE(lineOfSight(g, cm, {0, 3}, {3, 0}))
      << "Segment (0,3)->(3,0) passes through (1,2); must return false when (1,2) is lethal";
}

// Additional asymmetric: (0,4)->(4,1): dx=4, dy=-3.
// Integer-DDA: tDeltaX=6, tDeltaY=8, tMaxX=3, tMaxY=4.
// Step 1: tMaxX(3)<tMaxY(4): x=1, tMaxX=9. Check (1,4).
// Step 2: tMaxX(9)>tMaxY(4): y=3, tMaxY=12. Check (1,3).
// Step 3: tMaxX(9)<tMaxY(12): x=2, tMaxX=15. Check (2,3).
// Step 4: tMaxX(15)>tMaxY(12): y=2, tMaxY=20. Check (2,2).
// Step 5: tMaxX(15)<tMaxY(20): x=3, tMaxX=21. Check (3,2).
// Step 6: tMaxX(21)>tMaxY(20): y=1, tMaxY=28. Check (3,1).
// Step 7: tMaxX(21)<tMaxY(28): x=4. Check (4,1)=b. Done.
// Supercover: (0,4),(1,4),(1,3),(2,3),(2,2),(3,2),(3,1),(4,1).
// (2,2) is on the supercover.
TEST(PathSmoother, LineOfSight_Asymmetric_4_3_BlockedOnPath) {
  PlannerGrid g = makeOpen5();
  PlannerConfig cfg;
  CostModel cm(cfg);

  g.setAt(2, 2, static_cast<int8_t>(100));
  EXPECT_FALSE(lineOfSight(g, cm, {0, 4}, {4, 1}))
      << "Segment (0,4)->(4,1) passes through (2,2); must return false when (2,2) is lethal";
}

// Brute-force oracle: fixed segment list with hand-verified obstacle cells on
// the true supercover. Deterministic (fixed segment list, no RNG).
TEST(PathSmoother, LineOfSight_OracleCheck_FixedSegments) {
  PlannerGrid g = PlannerGrid(1.0, 0.0, 0.0, 10, 10, static_cast<int8_t>(0));
  PlannerConfig cfg;
  CostModel cm(cfg);

  struct SegTest { Cell a, b, obstacle; };
  const SegTest cases[] = {
    // non-diagonal asymmetric
    {{0, 2}, {4, 0}, {2, 1}},   // supercover includes (2,1)
    {{0, 1}, {3, 0}, {2, 0}},   // corner-cross: (2,0) is visited as main cell
    {{0, 4}, {4, 1}, {2, 2}},   // dx=4 dy=-3: (2,2) on path
    {{0, 0}, {6, 4}, {3, 2}},   // dx=6 dy=4: verify asymmetric mid-segment
    // pure diagonal (all corner crossings)
    {{0, 0}, {5, 5}, {3, 3}},   // diagonal: (3,3) on path
    {{0, 5}, {5, 0}, {2, 3}},   // anti-diagonal: corner-check hits (2,3)
  };

  for (const auto& tc : cases) {
    PlannerGrid gblock = g;
    gblock.setAt(tc.obstacle.x, tc.obstacle.y, static_cast<int8_t>(100));
    EXPECT_FALSE(lineOfSight(gblock, cm, tc.a, tc.b))
        << "Expected false for (" << tc.a.x << "," << tc.a.y
        << ")->(" << tc.b.x << "," << tc.b.y
        << ") with obstacle at (" << tc.obstacle.x << "," << tc.obstacle.y << ")";
    // Same segment on all-clear grid must return true.
    EXPECT_TRUE(lineOfSight(g, cm, tc.a, tc.b))
        << "Expected true for (" << tc.a.x << "," << tc.a.y
        << ")->(" << tc.b.x << "," << tc.b.y << ") on clear grid";
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
