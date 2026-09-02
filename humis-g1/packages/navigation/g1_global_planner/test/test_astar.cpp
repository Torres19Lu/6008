// A* unit tests: path quality, obstacle routing, edge cases, caps, corner-cut.

#include <gtest/gtest.h>

#include <cmath>
#include <algorithm>

#include "g1_global_planner/core/cost_model.h"
#include "g1_global_planner/core/plan_types.h"
#include "g1_global_planner/core/planner_grid.h"
#include "g1_global_planner/planner/astar.h"

using g1_global_planner::AStarResult;
using g1_global_planner::Cell;
using g1_global_planner::CostModel;
using g1_global_planner::PlannerConfig;
using g1_global_planner::PlannerGrid;
using g1_global_planner::PlanStatus;

// Helper: make a 10x10 all-free grid.
static PlannerGrid makeOpen() {
  return PlannerGrid(0.1, 0.0, 0.0, 10, 10, static_cast<int8_t>(0));
}

TEST(AStar, StraightPathLength) {
  PlannerGrid g = makeOpen();
  PlannerConfig cfg;
  CostModel cm(cfg);
  // Straight horizontal: (0,0) -> (5,0), optimal cost = 5 moves * stepCost(0)=1.
  AStarResult r = astar(g, cm, cfg, {0, 0}, {5, 0});
  ASSERT_EQ(r.status, PlanStatus::SUCCESS);
  EXPECT_EQ(r.path.front(), (Cell{0, 0}));
  EXPECT_EQ(r.path.back(),  (Cell{5, 0}));
  // 6 cells, 5 steps.
  EXPECT_EQ(static_cast<int>(r.path.size()), 6);
}

TEST(AStar, DiagonalPathLength) {
  PlannerGrid g = makeOpen();
  PlannerConfig cfg;
  cfg.allow_corner_cutting = true;
  CostModel cm(cfg);
  // Diagonal: (0,0) -> (5,5). Optimal = 5 diagonal moves, cost = 5*sqrt(2).
  AStarResult r = astar(g, cm, cfg, {0, 0}, {5, 5});
  ASSERT_EQ(r.status, PlanStatus::SUCCESS);
  EXPECT_EQ(r.path.front(), (Cell{0, 0}));
  EXPECT_EQ(r.path.back(),  (Cell{5, 5}));
  EXPECT_EQ(static_cast<int>(r.path.size()), 6);
  // Each step should be diagonal.
  for (int i = 0; i + 1 < static_cast<int>(r.path.size()); ++i) {
    EXPECT_EQ(std::abs(r.path[i+1].x - r.path[i].x), 1);
    EXPECT_EQ(std::abs(r.path[i+1].y - r.path[i].y), 1);
  }
}

TEST(AStar, VerticalWallWithGap) {
  // 10x10 grid, vertical wall at x=5, blocked from y=1..9, gap at y=0.
  PlannerGrid g = makeOpen();
  for (int y = 1; y < 10; ++y) g.setAt(5, y, static_cast<int8_t>(100));
  PlannerConfig cfg;
  CostModel cm(cfg);
  // Route from (0,5) to (9,5): must go through gap at (5,0).
  AStarResult r = astar(g, cm, cfg, {0, 5}, {9, 5});
  ASSERT_EQ(r.status, PlanStatus::SUCCESS);
  // Path must cross x=5 through y=0 (the gap).
  bool crossed_gap = false;
  for (const Cell& c : r.path) {
    if (c.x == 5 && c.y == 0) { crossed_gap = true; break; }
  }
  EXPECT_TRUE(crossed_gap) << "Path did not pass through the gap at y=0";
}

TEST(AStar, GoalWalledOffReturnsNoPath) {
  PlannerGrid g = makeOpen();
  // Completely surround (5,5).
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
      if (!(dx == 0 && dy == 0))
        g.setAt(5 + dx, 5 + dy, static_cast<int8_t>(100));
  g.setAt(5, 5, static_cast<int8_t>(0));
  PlannerConfig cfg;
  CostModel cm(cfg);
  // Goal is unreachable from (0,0) because all 8 neighbors are blocked.
  AStarResult r = astar(g, cm, cfg, {0, 0}, {5, 5});
  EXPECT_EQ(r.status, PlanStatus::NO_PATH);
}

TEST(AStar, BlockedStartReturnsBadStart) {
  PlannerGrid g = makeOpen();
  g.setAt(0, 0, static_cast<int8_t>(100));
  PlannerConfig cfg;
  CostModel cm(cfg);
  AStarResult r = astar(g, cm, cfg, {0, 0}, {9, 9});
  EXPECT_EQ(r.status, PlanStatus::BAD_START);
}

TEST(AStar, BlockedGoalReturnsBadGoal) {
  PlannerGrid g = makeOpen();
  g.setAt(9, 9, static_cast<int8_t>(100));
  PlannerConfig cfg;
  CostModel cm(cfg);
  AStarResult r = astar(g, cm, cfg, {0, 0}, {9, 9});
  EXPECT_EQ(r.status, PlanStatus::BAD_GOAL);
}

TEST(AStar, UnknownBandBlocksWithoutAllowUnknown) {
  PlannerGrid g = makeOpen();
  // Band of unknown (-1) cells at x=5.
  for (int y = 0; y < 10; ++y) g.setAt(5, y, static_cast<int8_t>(-1));
  PlannerConfig cfg;
  cfg.allow_unknown = false;
  CostModel cm(cfg);
  AStarResult r = astar(g, cm, cfg, {0, 5}, {9, 5});
  EXPECT_EQ(r.status, PlanStatus::NO_PATH);
}

TEST(AStar, UnknownBandCrossedWithAllowUnknown) {
  PlannerGrid g = makeOpen();
  for (int y = 0; y < 10; ++y) g.setAt(5, y, static_cast<int8_t>(-1));
  PlannerConfig cfg;
  cfg.allow_unknown = true;
  CostModel cm(cfg);
  AStarResult r = astar(g, cm, cfg, {0, 5}, {9, 5});
  EXPECT_EQ(r.status, PlanStatus::SUCCESS);
}

TEST(AStar, MaxExpansionsCap) {
  PlannerGrid g = makeOpen();
  PlannerConfig cfg;
  cfg.max_expansions = 3;
  CostModel cm(cfg);
  // Long route (0,0) -> (9,9) on a 10x10 open grid will need many expansions.
  AStarResult r = astar(g, cm, cfg, {0, 0}, {9, 9});
  EXPECT_EQ(r.status, PlanStatus::CAPPED);
  EXPECT_GT(r.expansions, 3);
}

TEST(AStar, NoCornercutBlocksDiagonal) {
  // Obstacles at (4,3) and (3,4) form a diagonally-touching pair.
  // A path from (3,3) to (5,5) would ideally cut through (4,4) diagonally from
  // (3,3), but (4,3) and (3,4) are the two shared orthogonals for that step.
  // With allow_corner_cutting=false the step (3,3)->(4,4) is forbidden.
  // The path must go around: e.g., (3,3)->(5,3) or (3,3)->(3,5) and then to (5,5).
  PlannerGrid g = makeOpen();
  g.setAt(4, 3, static_cast<int8_t>(100));
  g.setAt(3, 4, static_cast<int8_t>(100));

  // --- Part A: allow_corner_cutting=false must suppress the (3,3)->(4,4) step ---
  {
    PlannerConfig cfg;
    cfg.allow_corner_cutting = false;
    CostModel cm(cfg);
    AStarResult r = astar(g, cm, cfg, {3, 3}, {5, 5});
    ASSERT_EQ(r.status, PlanStatus::SUCCESS);
    bool has_shortcut = false;
    for (int i = 0; i + 1 < static_cast<int>(r.path.size()); ++i) {
      if (r.path[i].x == 3 && r.path[i].y == 3 &&
          r.path[i+1].x == 4 && r.path[i+1].y == 4) {
        has_shortcut = true;
        break;
      }
    }
    EXPECT_FALSE(has_shortcut) << "Path cut the corner between blocked (4,3) and (3,4)";
  }

  // --- Part B: allow_corner_cutting=true must permit the (3,3)->(4,4) step ---
  // This proves the rule is what suppresses the diagonal, not some unrelated reason.
  {
    PlannerConfig cfg;
    cfg.allow_corner_cutting = true;
    CostModel cm(cfg);
    AStarResult r = astar(g, cm, cfg, {3, 3}, {5, 5});
    ASSERT_EQ(r.status, PlanStatus::SUCCESS);
    bool has_shortcut = false;
    for (int i = 0; i + 1 < static_cast<int>(r.path.size()); ++i) {
      if (r.path[i].x == 3 && r.path[i].y == 3 &&
          r.path[i+1].x == 4 && r.path[i+1].y == 4) {
        has_shortcut = true;
        break;
      }
    }
    EXPECT_TRUE(has_shortcut)
        << "With allow_corner_cutting=true the optimal (3,3)->(4,4) diagonal "
        << "step must appear in the path";
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
