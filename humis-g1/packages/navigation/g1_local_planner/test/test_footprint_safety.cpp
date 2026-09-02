// g1_local_planner/test/test_footprint_safety.cpp
#include <gtest/gtest.h>
#include <vector>
#include "g1_local_planner/core/footprint_safety.h"
#include "g1_local_planner/core/robot_footprint.h"
#include "g1_local_planner/core/costmap_view.h"

using namespace g1_local_planner;

// Build a costmap: free (0) everywhere except a vertical wall at column wx_cell with
// a gap of `gap_cells` rows centered vertically. resolution 0.05, origin 0,0.
static CostmapView makeWallWithGap(int w, int h, int wall_col, int gap_cells) {
  std::vector<int8_t> data(static_cast<size_t>(w * h), 0);
  const int mid = h / 2;
  for (int y = 0; y < h; ++y) {
    const bool in_gap = std::abs(y - mid) * 2 < gap_cells;  // ~gap_cells tall
    if (!in_gap) data[static_cast<size_t>(y * w + wall_col)] = 100;
  }
  return CostmapView(0.05, 0.0, 0.0, w, h, data, 1.0, true);
}

static RobotFootprint footprint() {
  FootprintConfig cfg;  // r=0.15, W=0.5 -> circumscribed 0.25
  RobotFootprint fp; std::string err;
  RobotFootprint::build(cfg, &fp, &err);
  return fp;
}

TEST(FootprintSafety, ClearWhenAwayFromWall) {
  CostmapView cm = makeWallWithGap(40, 40, 30, 6);
  FootprintSafety fs(footprint(), SafetyConfig{});
  Pose2D p; p.x = 0.5; p.y = 1.0; p.yaw = 0.0;  // far from the wall column
  EXPECT_FALSE(fs.poseInCollision(p, cm));
}

TEST(FootprintSafety, NarrowGapPassableDependsOnYaw) {
  // The faithful disk check tests each circle's FULL AREA (radius 0.15), so a
  // 0.5 m wide x 0.3 m deep body needs the gap to exceed the dimension facing it.
  // makeWallWithGap(.,.,.,8) opens rows |y-mid| < 4 -> 7 free rows -> 0.35 m gap.
  // yaw=pi/2 presents the 0.30 m DEPTH (fits 0.35 m -> CLEARS); yaw=0 presents the
  // 0.50 m WIDTH (does not fit -> COLLIDES). This is the orientation-aware check the
  // old center-only test could not express (it wrongly passed a 0.20 m gap at pi/2).
  CostmapView cm = makeWallWithGap(80, 80, 40, 8);  // 0.35 m gap
  FootprintSafety fs(footprint(), SafetyConfig{});   // footprint_collision_margin 0.0
  Pose2D at_gap;
  at_gap.x = (40 + 0.5) * 0.05;  // center of wall column
  at_gap.y = (40 + 0.5) * 0.05;  // center of gap (mid row)
  Pose2D wide = at_gap; wide.yaw = 0.0;         // 0.5 m width faces the gap -> collides
  Pose2D deep = at_gap; deep.yaw = M_PI / 2.0;  // 0.3 m depth faces the gap -> clears
  EXPECT_TRUE(fs.poseInCollision(wide, cm));
  EXPECT_FALSE(fs.poseInCollision(deep, cm));
}

// Inscribed (99) is NOT a collision: the faithful check uses lethal_value=100
// (true-lethal only), so the footprint may sit on inscribed cells (close approach).
TEST(FootprintSafety, InscribedIsNotCollision) {
  std::vector<int8_t> data(static_cast<size_t>(40 * 40), 99);  // all inscribed
  CostmapView cm(0.05, 0.0, 0.0, 40, 40, data, 1.0, true);
  FootprintSafety fs(footprint(), SafetyConfig{});  // lethal_value default 100
  Pose2D p; p.x = 1.0; p.y = 1.0; p.yaw = 0.0;
  EXPECT_FALSE(fs.poseInCollision(p, cm));
}

// True-lethal under a footprint DISK (not just a center) is a collision: a lethal
// cell 0.10 m off-center (inside the 0.15 m circle, but NOT at any circle center)
// must be caught -- the area check, not the old center-only check.
TEST(FootprintSafety, LethalUnderDiskOffCenterIsCollision) {
  const double res = 0.05;
  std::vector<int8_t> data(static_cast<size_t>(60 * 60), 0);
  CostmapView base(res, 0.0, 0.0, 60, 60, data, 1.0, true);
  // Robot center at world (1.5, 1.5), yaw 0. Put a lethal cell 0.10 m ahead in +x
  // (inside the center circle radius 0.15, but no circle center sits there).
  int cx, cy;
  ASSERT_TRUE(base.worldToMap(1.5 + 0.10, 1.5, cx, cy));
  data[static_cast<size_t>(cy * 60 + cx)] = 100;
  CostmapView cm(res, 0.0, 0.0, 60, 60, data, 1.0, true);
  FootprintSafety fs(footprint(), SafetyConfig{});
  Pose2D p; p.x = 1.5; p.y = 1.5; p.yaw = 0.0;
  EXPECT_TRUE(fs.poseInCollision(p, cm));
}

TEST(FootprintSafety, ClearDistanceFindsApproachingWall) {
  CostmapView cm = makeWallWithGap(80, 20, 60, 0);  // solid wall, no gap
  FootprintSafety fs(footprint(), SafetyConfig{});
  Pose2D current; current.x = 0.5; current.y = 0.5; current.yaw = 0.0;
  // Predicted poses marching toward +x into the wall at column 60 (x=3.0 m).
  std::vector<Pose2D> pred;
  for (int k = 1; k <= 20; ++k) { Pose2D s; s.x = 0.5 + 0.15 * k; s.y = 0.5; s.yaw = 0.0; pred.push_back(s); }
  const double d = fs.clearDistance(current, pred, cm);
  EXPECT_LT(d, FootprintSafety::kClearInf);
  EXPECT_GT(d, 0.0);
  EXPECT_LT(d, 3.0);  // collision before reaching the wall plane
}

TEST(FootprintSafety, ClearDistanceInfiniteWhenClear) {
  CostmapView cm = makeWallWithGap(80, 40, 79, 40);  // wall off to the side, full gap
  FootprintSafety fs(footprint(), SafetyConfig{});
  Pose2D current; current.x = 0.5; current.y = 1.0; current.yaw = 0.0;
  std::vector<Pose2D> pred;
  for (int k = 1; k <= 10; ++k) { Pose2D s; s.x = 0.5 + 0.05 * k; s.y = 1.0; s.yaw = 0.0; pred.push_back(s); }
  EXPECT_GE(fs.clearDistance(current, pred, cm), FootprintSafety::kClearInf);
}

// ---------------------------------------------------------------------------
// SafetyResult + filter tests
// ---------------------------------------------------------------------------
#include "g1_local_planner/core/plan_types.h"

TEST(FootprintSafety, PassesWhenClear) {
  CostmapView cm = makeWallWithGap(80, 40, 79, 40);
  FootprintSafety fs(footprint(), SafetyConfig{});
  Pose2D cur; cur.x = 0.5; cur.y = 1.0;
  MpcResult m; m.applied = Twist2D{0.4, 0.0, 0.0};
  for (int k = 1; k <= 10; ++k) { Pose2D s; s.x = 0.5 + 0.04 * k; s.y = 1.0; m.predicted.push_back(s); }
  SafetyResult r = fs.filter(cur, m, cm, 1.5, 0.05);
  EXPECT_FALSE(r.intervened);
  EXPECT_NEAR(r.command.vx, 0.4, 1e-9);
  EXPECT_EQ(r.state, ControllerState::TRACKING);
}

TEST(FootprintSafety, SlowsNearWall) {
  // Wall at cell column 11 (x = 0.55 m). Robot at x = 0.5 (mx = 10, clear).
  // First predicted hit: between step 1 (x=0.54, mx=10) and step 2 (x=0.58, mx=11).
  // Sub=2: t=0.5 gives x=0.56 -> mx=11 -> collision. d = 0.04 + 0.5*0.04 = 0.06 m.
  // d_usable = 0.06 - 0.05 = 0.01 m. v_safe = sqrt(2*1.5*0.01) = 0.173 m/s < 0.5 m/s.
  CostmapView cm = makeWallWithGap(40, 20, 11, 0);
  FootprintSafety fs(footprint(), SafetyConfig{});
  Pose2D cur; cur.x = 0.5; cur.y = 0.5;
  MpcResult m; m.applied = Twist2D{0.5, 0.0, 0.0};
  for (int k = 1; k <= 20; ++k) { Pose2D s; s.x = 0.5 + 0.04 * k; s.y = 0.5; m.predicted.push_back(s); }
  SafetyResult r = fs.filter(cur, m, cm, 1.5, 0.05);
  EXPECT_TRUE(r.intervened);
  EXPECT_LT(r.command.vx, 0.5);   // scaled down
  EXPECT_GE(r.command.vx, 0.0);
}

TEST(FootprintSafety, StopsAtWall) {
  // Wall at cell column 10 (x = 0.50 m). Robot at x = 0.5 -> circle center maps to
  // mx = floor(0.5/0.05) = 10, which is the wall column. poseInCollision(current) = true.
  // Filter returns zero command immediately (hard stop at the wall).
  CostmapView cm = makeWallWithGap(40, 20, 10, 0);
  FootprintSafety fs(footprint(), SafetyConfig{});
  Pose2D cur; cur.x = 0.5; cur.y = 0.5;
  MpcResult m; m.applied = Twist2D{0.5, 0.0, 0.0};
  for (int k = 1; k <= 20; ++k) { Pose2D s; s.x = 0.5 + 0.04 * k; s.y = 0.5; m.predicted.push_back(s); }
  SafetyResult r = fs.filter(cur, m, cm, 1.5, 0.05);
  EXPECT_TRUE(r.intervened);
  EXPECT_NEAR(r.command.vx, 0.0, 0.08);  // braking to zero (current pose at wall)
}

// ---------------------------------------------------------------------------
// Unknown-cell hardening
// ---------------------------------------------------------------------------
TEST(FootprintSafety, UnknownCellIsCollisionWhenConfigured) {
  // Build a costmap whose footprint-covered cells are value -1 (unknown).
  // Footprint circles land near center (x=0.5, y=0.5). Resolution 0.05,
  // origin 0,0, so center = cell (10,10) in a 20x20 grid.
  // Fill all cells with -1 (unknown) as int8_t.
  const int w = 20, h = 20;
  std::vector<int8_t> data(static_cast<size_t>(w * h), -1);
  CostmapView cm(0.05, 0.0, 0.0, w, h, data, 1.0, true);

  // Default config: treat_unknown_as_obstacle = true -> collision expected.
  SafetyConfig cfg_default{};
  FootprintSafety fs_default(footprint(), cfg_default);
  Pose2D p; p.x = 0.5; p.y = 0.5; p.yaw = 0.0;
  EXPECT_TRUE(fs_default.poseInCollision(p, cm));

  // treat_unknown_as_obstacle = false and no lethal cells -> no collision.
  SafetyConfig cfg_permissive{};
  cfg_permissive.treat_unknown_as_obstacle = false;
  FootprintSafety fs_permissive(footprint(), cfg_permissive);
  EXPECT_FALSE(fs_permissive.poseInCollision(p, cm));
}

// The swept stride is bound to the costmap resolution, so a thin (one-cell) wall is
// NOT tunneled even with a coarse swept_subsamples and a large single predicted step.
TEST(FootprintSafety, ClearDistanceDoesNotTunnelThinWallWithCoarseSubsamples) {
  // resolution 0.05; a 1-cell-wide solid wall at column 20 -> world x ~ 1.0 m.
  CostmapView cm = makeWallWithGap(60, 20, 20, 0);
  SafetyConfig sc{};
  sc.swept_subsamples = 1;  // coarse: t=1 only would jump past the wall (tunnel)
  FootprintSafety fs(footprint(), sc);
  Pose2D current; current.x = 0.5; current.y = 0.5; current.yaw = 0.0;
  // One large predicted step that lands BEYOND the wall (x 0.5 -> 2.0).
  std::vector<Pose2D> pred;
  Pose2D s; s.x = 2.0; s.y = 0.5; s.yaw = 0.0; pred.push_back(s);
  const double d = fs.clearDistance(current, pred, cm);
  EXPECT_LT(d, FootprintSafety::kClearInf);  // the thin wall is detected, not skipped
  EXPECT_GT(d, 0.0);
  EXPECT_LT(d, 1.0);                         // collision at/before the wall plane (~0.5 m)
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
