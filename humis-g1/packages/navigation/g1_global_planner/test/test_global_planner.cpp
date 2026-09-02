// GlobalPlanner end-to-end tests: world path, snapping, yaw, pathStillValid.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "g1_global_planner/core/cost_model.h"
#include "g1_global_planner/core/plan_types.h"
#include "g1_global_planner/core/planner_grid.h"
#include "g1_global_planner/planner/global_planner.h"
#include "g1_global_planner/planner/path_smoother.h"

using g1_global_planner::Cell;
using g1_global_planner::CostModel;
using g1_global_planner::GlobalPlanner;
using g1_global_planner::PlannerConfig;
using g1_global_planner::PlannerGrid;
using g1_global_planner::PlanResult;
using g1_global_planner::PlanStatus;
using g1_global_planner::Pose2D;
using g1_global_planner::lineOfSight;

// 20x20 grid, res=0.5, origin=(0,0): spans [0,10)x[0,10) world.
static PlannerGrid makeGrid20() {
  return PlannerGrid(0.5, 0.0, 0.0, 20, 20, static_cast<int8_t>(0));
}

TEST(GlobalPlanner, EndToEndSuccessCollisionFree) {
  PlannerGrid g = makeGrid20();
  PlannerConfig cfg;
  cfg.smooth_enable = true;
  GlobalPlanner planner(cfg);

  Pose2D start{0.25, 0.25, 0.0};
  Pose2D goal {8.75, 8.75, 1.57};

  PlanResult r = planner.plan(g, start, goal);
  ASSERT_EQ(r.status, PlanStatus::SUCCESS);
  ASSERT_FALSE(r.path.empty());

  // First point near start, last near goal.
  EXPECT_NEAR(r.path.front().x, 0.25, 0.5);
  EXPECT_NEAR(r.path.front().y, 0.25, 0.5);
  EXPECT_NEAR(r.path.back().x,  8.75, 0.5);
  EXPECT_NEAR(r.path.back().y,  8.75, 0.5);

  // Last waypoint yaw == goal.yaw.
  EXPECT_NEAR(r.path.back().yaw, 1.57, 1e-6);

  // All waypoints in-bounds.
  for (const Pose2D& p : r.path) {
    EXPECT_GE(p.x, 0.0);
    EXPECT_LT(p.x, 10.0);
    EXPECT_GE(p.y, 0.0);
    EXPECT_LT(p.y, 10.0);
  }

  // Yaws along intermediate waypoints point from current to next.
  for (std::size_t i = 0; i + 1 < r.path.size(); ++i) {
    double expected_yaw = std::atan2(r.path[i+1].y - r.path[i].y,
                                     r.path[i+1].x - r.path[i].x);
    EXPECT_NEAR(r.path[i].yaw, expected_yaw, 1e-6)
        << "Yaw mismatch at waypoint " << i;
  }

  // length_m > 0.
  EXPECT_GT(r.length_m, 0.0);
}

TEST(GlobalPlanner, BlockedGoalWithinSnapRadiusSnapsSuccess) {
  PlannerGrid g = makeGrid20();
  // Block the cell containing (5.0, 5.0): cell (10,10).
  g.setAt(10, 10, static_cast<int8_t>(100));

  PlannerConfig cfg;
  cfg.goal_snap_radius = 0.6;  // 0.6/0.5 = 1.2 -> snap_cells=1
  GlobalPlanner planner(cfg);

  Pose2D start{0.25, 0.25, 0.0};
  Pose2D goal {5.0, 5.0, 0.0};  // maps to blocked (10,10)

  PlanResult r = planner.plan(g, start, goal);
  EXPECT_EQ(r.status, PlanStatus::SUCCESS);
}

TEST(GlobalPlanner, BlockedGoalOutsideSnapRadiusReturnsBadGoal) {
  PlannerGrid g = makeGrid20();
  // Block a 3x3 region around (5,5): cells (9..11, 9..11).
  for (int dy = -1; dy <= 1; ++dy)
    for (int dx = -1; dx <= 1; ++dx)
      g.setAt(10 + dx, 10 + dy, static_cast<int8_t>(100));

  PlannerConfig cfg;
  cfg.goal_snap_radius = 0.1;  // 0.1/0.5 = 0.2 -> snap_cells=0 (no snap)
  GlobalPlanner planner(cfg);

  Pose2D start{0.25, 0.25, 0.0};
  Pose2D goal {5.0, 5.0, 0.0};

  PlanResult r = planner.plan(g, start, goal);
  EXPECT_EQ(r.status, PlanStatus::BAD_GOAL);
}

TEST(GlobalPlanner, PathStillValidOnClearGrid) {
  PlannerGrid g = makeGrid20();
  PlannerConfig cfg;
  GlobalPlanner planner(cfg);

  Pose2D start{0.25, 0.25, 0.0};
  Pose2D goal {8.75, 8.75, 0.0};
  PlanResult r = planner.plan(g, start, goal);
  ASSERT_EQ(r.status, PlanStatus::SUCCESS);

  EXPECT_TRUE(planner.pathStillValid(g, r.path));
}

TEST(GlobalPlanner, PathStillValidFailsAfterObstacle) {
  PlannerGrid g = makeGrid20();
  PlannerConfig cfg;
  GlobalPlanner planner(cfg);

  Pose2D start{0.25, 0.25, 0.0};
  Pose2D goal {8.75, 8.75, 0.0};
  PlanResult r = planner.plan(g, start, goal);
  ASSERT_EQ(r.status, PlanStatus::SUCCESS);
  ASSERT_GE(static_cast<int>(r.path.size()), 2);

  // Block a cell that the second waypoint maps to.
  int mx, my;
  bool ok = g.worldToMap(r.path[1].x, r.path[1].y, mx, my);
  ASSERT_TRUE(ok);
  g.setAt(mx, my, static_cast<int8_t>(100));

  EXPECT_FALSE(planner.pathStillValid(g, r.path));
}

TEST(GlobalPlanner, EmptyAndSinglePointPathAlwaysValid) {
  PlannerGrid g = makeGrid20();
  PlannerConfig cfg;
  GlobalPlanner planner(cfg);

  EXPECT_TRUE(planner.pathStillValid(g, {}));
  EXPECT_TRUE(planner.pathStillValid(g, {{1.0, 1.0, 0.0}}));
}

// Finding 4: end-to-end collision-free assertion on a grid with lethal obstacles.
// The obstacle is placed near the straight start->goal line so that a buggy
// lineOfSight implementation would fail to detect it as a blocker.
// Grid: 20x20, res=0.5m, world spans [0,10)x[0,10).
// Start=(0.25,0.25) -> cell(0,0).  Goal=(8.75,8.75) -> cell(17,17).
// The diagonal line from (0,0) to (17,17) passes through cell (8,8) (world
// center ~(4.25,4.25)).  We place a lethal cell at (8,8) to force the planner
// around it.  The returned path must:
//   - have every waypoint in a non-lethal cell, and
//   - have every consecutive waypoint pair pass lineOfSight on the obstacle grid.
TEST(GlobalPlanner, EndToEndCollisionFreeWithObstacle) {
  PlannerGrid g = makeGrid20();
  // Place a lethal block at cell (8,8) -- on the straight diagonal from start to goal.
  g.setAt(8, 8, static_cast<int8_t>(100));

  PlannerConfig cfg;
  cfg.smooth_enable = true;
  cfg.goal_snap_radius = 0.3;
  GlobalPlanner planner(cfg);
  CostModel cost(cfg);

  Pose2D start{0.25, 0.25, 0.0};
  Pose2D goal {8.75, 8.75, 0.0};

  PlanResult r = planner.plan(g, start, goal);
  ASSERT_EQ(r.status, PlanStatus::SUCCESS);
  ASSERT_FALSE(r.path.empty());

  // Every waypoint must map to a non-blocked (non-lethal) cell.
  for (std::size_t i = 0; i < r.path.size(); ++i) {
    int mx, my;
    bool in = g.worldToMap(r.path[i].x, r.path[i].y, mx, my);
    EXPECT_TRUE(in) << "Waypoint " << i << " is out of bounds";
    if (in) {
      EXPECT_FALSE(cost.blocked(g.at(mx, my)))
          << "Waypoint " << i << " at cell (" << mx << "," << my
          << ") maps to a blocked cell";
    }
  }

  // Every consecutive waypoint pair must pass lineOfSight on this grid.
  for (std::size_t i = 0; i + 1 < r.path.size(); ++i) {
    int ax, ay, bx, by;
    ASSERT_TRUE(g.worldToMap(r.path[i].x,     r.path[i].y,     ax, ay));
    ASSERT_TRUE(g.worldToMap(r.path[i+1].x,   r.path[i+1].y,   bx, by));
    EXPECT_TRUE(lineOfSight(g, cost, {ax, ay}, {bx, by}))
        << "Consecutive waypoints " << i << " and " << i+1
        << " fail lineOfSight on the obstacle grid";
  }
}

// Finding 7: goal_snap_radius == 0.0 -> snapping is OFF; blocked goal returns BAD_GOAL.
TEST(GlobalPlanner, SnapRadiusZeroBlockedGoalReturnsBadGoal) {
  PlannerGrid g = makeGrid20();
  // Block the cell that (5.0, 5.0) maps to: cell (10,10).
  g.setAt(10, 10, static_cast<int8_t>(100));

  PlannerConfig cfg;
  cfg.goal_snap_radius = 0.0;  // snapping disabled
  GlobalPlanner planner(cfg);

  Pose2D start{0.25, 0.25, 0.0};
  Pose2D goal {5.0, 5.0, 0.0};

  PlanResult r = planner.plan(g, start, goal);
  EXPECT_EQ(r.status, PlanStatus::BAD_GOAL)
      << "With goal_snap_radius=0.0 a blocked goal must return BAD_GOAL (no snapping)";
}

// ---------------------------------------------------------------------------
// Goal relax: an INSCRIBED (99) goal cell inside goal_relax_radius is reachable
// (relaxed to traversable), but a LETHAL (100) cell is never entered.
// ---------------------------------------------------------------------------
TEST(GlobalPlanner, GoalRelaxReachesInscribedGoal) {
  const double res = 0.1;
  const int W = 30, H = 30;
  std::vector<int8_t> data(static_cast<size_t>(W * H), 0);
  auto idx = [&](int x, int y) { return static_cast<size_t>(y * W + x); };
  // Lethal obstacle at cell (20,15); inscribed ring (99) in the 3-cell neighborhood.
  data[idx(20, 15)] = 100;
  for (int dy = -3; dy <= 3; ++dy)
    for (int dx = -3; dx <= 3; ++dx) {
      if (dx == 0 && dy == 0) continue;
      const int x = 20 + dx, y = 15 + dy;
      if (x >= 0 && x < W && y >= 0 && y < H && data[idx(x, y)] == 0)
        data[idx(x, y)] = 99;
    }
  PlannerGrid grid(res, 0.0, 0.0, W, H, data);

  PlannerConfig cfg;             // lethal_threshold=99, goal_relax_enable=true
  cfg.goal_relax_radius = 0.6;   // 6 cells, covers the inscribed ring
  GlobalPlanner gp(cfg);

  Pose2D start{0.2, 1.5, 0.0};
  // Goal on an INSCRIBED cell just outside the lethal one (cell (18,15) -> world).
  Pose2D goal{(18 + 0.5) * res, (15 + 0.5) * res, 0.0};
  const PlanResult r = gp.plan(grid, start, goal);
  ASSERT_EQ(r.status, PlanStatus::SUCCESS);
  // The path endpoint reaches the inscribed goal cell (not snapped ~0.3 m away).
  const Pose2D end = r.path.back();
  EXPECT_NEAR(end.x, goal.x, res);
  EXPECT_NEAR(end.y, goal.y, res);
  // And it never lands on the lethal cell.
  for (const Pose2D& p : r.path) {
    int mx, my;
    ASSERT_TRUE(grid.worldToMap(p.x, p.y, mx, my));
    EXPECT_FALSE(mx == 20 && my == 15) << "path entered the lethal cell";
  }
}

// ---------------------------------------------------------------------------
// Travel still blocks inscribed: with relax DISABLED, an inscribed goal snaps out
// (legacy behaviour preserved; lethal_threshold=99 blocks 99).
// ---------------------------------------------------------------------------
TEST(GlobalPlanner, TravelBlocksInscribedWhenRelaxOff) {
  const double res = 0.1;
  const int W = 30, H = 30;
  std::vector<int8_t> data(static_cast<size_t>(W * H), 0);
  auto idx = [&](int x, int y) { return static_cast<size_t>(y * W + x); };
  data[idx(20, 15)] = 100;
  for (int dy = -3; dy <= 3; ++dy)
    for (int dx = -3; dx <= 3; ++dx) {
      const int x = 20 + dx, y = 15 + dy;
      if (!(dx == 0 && dy == 0) && data[idx(x, y)] == 0) data[idx(x, y)] = 99;
    }
  PlannerGrid grid(res, 0.0, 0.0, W, H, data);

  PlannerConfig cfg;
  cfg.goal_relax_enable = false;     // legacy
  GlobalPlanner gp(cfg);
  Pose2D start{0.2, 1.5, 0.0};
  Pose2D goal{(18 + 0.5) * res, (15 + 0.5) * res, 0.0};  // inscribed cell
  const PlanResult r = gp.plan(grid, start, goal);
  ASSERT_EQ(r.status, PlanStatus::SUCCESS);
  // Endpoint snapped OFF the inscribed ring (>= ~0.25 m from the lethal cell).
  EXPECT_GT(std::hypot(r.path.back().x - (20 + 0.5) * res,
                       r.path.back().y - (15 + 0.5) * res), 0.25);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
