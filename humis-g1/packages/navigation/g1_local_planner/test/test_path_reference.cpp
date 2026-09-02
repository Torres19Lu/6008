// Unit tests for path_reference: projectOntoPath, pointAtArcLength, generateReference.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "g1_local_planner/planner/path_reference.h"
#include "g1_local_planner/core/plan_types.h"

using g1_local_planner::MpcConfig;
using g1_local_planner::PathProjection;
using g1_local_planner::Pose2D;
using g1_local_planner::generateReference;
using g1_local_planner::pointAtArcLength;
using g1_local_planner::projectOntoPath;

static constexpr double kTol = 1e-6;

// Helper: build a Pose2D with the given x, y (yaw stays 0).
static Pose2D makeXY(double x, double y) {
  Pose2D p;
  p.x = x;
  p.y = y;
  return p;
}

// Helper: straight path along +x from 0 to length with n_pts points.
static std::vector<Pose2D> straightPath(double length, int n_pts) {
  std::vector<Pose2D> path;
  path.reserve(static_cast<size_t>(n_pts));
  for (int i = 0; i < n_pts; ++i) {
    path.push_back(makeXY(length * static_cast<double>(i) / static_cast<double>(n_pts - 1), 0.0));
  }
  return path;
}

// ---------------------------------------------------------------------------
// projectOntoPath

TEST(ProjectOntoPath, RobotOnStraightPath) {
  // Path along +x from 0 to 1 (two waypoints).
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  PathProjection p = projectOntoPath(path, 0.35, 0.0);

  EXPECT_TRUE(p.valid);
  EXPECT_NEAR(p.arc_length, 0.35, kTol);
  EXPECT_NEAR(p.point.x,    0.35, kTol);
  EXPECT_NEAR(p.point.y,    0.0,  kTol);
  EXPECT_NEAR(p.tangent_yaw, 0.0, kTol);  // segment along +x
  EXPECT_EQ(p.seg, static_cast<size_t>(0));
}

TEST(ProjectOntoPath, RobotOffsetFromStraightPath) {
  // Path along +x; robot is at (0.35, 0.2) -- perpendicular offset.
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  PathProjection p = projectOntoPath(path, 0.35, 0.2);

  EXPECT_TRUE(p.valid);
  EXPECT_NEAR(p.arc_length, 0.35, kTol);  // projection is still at x=0.35
  EXPECT_NEAR(p.point.x,    0.35, kTol);
  EXPECT_NEAR(p.point.y,    0.0,  kTol);
  EXPECT_NEAR(p.tangent_yaw, 0.0, kTol);
}

TEST(ProjectOntoPath, EmptyPath) {
  std::vector<Pose2D> path;
  PathProjection p = projectOntoPath(path, 1.0, 2.0);
  EXPECT_FALSE(p.valid);
}

TEST(ProjectOntoPath, SinglePointPath) {
  std::vector<Pose2D> path = { makeXY(3.0, 4.0) };
  PathProjection p = projectOntoPath(path, 1.0, 2.0);

  EXPECT_TRUE(p.valid);
  EXPECT_NEAR(p.arc_length,  0.0, kTol);
  EXPECT_NEAR(p.point.x,     3.0, kTol);
  EXPECT_NEAR(p.point.y,     4.0, kTol);
  EXPECT_NEAR(p.tangent_yaw, 0.0, kTol);
  EXPECT_EQ(p.seg, static_cast<size_t>(0));
}

TEST(ProjectOntoPath, RobotBeforePathStart) {
  // Robot is behind the path start -- projection clamps to the first waypoint.
  std::vector<Pose2D> path = { makeXY(1.0, 0.0), makeXY(2.0, 0.0) };
  PathProjection p = projectOntoPath(path, 0.0, 0.0);

  EXPECT_TRUE(p.valid);
  EXPECT_NEAR(p.arc_length, 0.0, kTol);  // clamped to segment start
  EXPECT_NEAR(p.point.x,    1.0, kTol);
  EXPECT_NEAR(p.point.y,    0.0, kTol);
}

TEST(ProjectOntoPath, RobotPastPathEnd) {
  // Robot is beyond the last waypoint -- projection clamps to the last waypoint.
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  PathProjection p = projectOntoPath(path, 2.0, 0.0);

  EXPECT_TRUE(p.valid);
  EXPECT_NEAR(p.point.x, 1.0, kTol);
  EXPECT_NEAR(p.point.y, 0.0, kTol);
  // arc_length should equal the total path length (1.0).
  EXPECT_NEAR(p.arc_length, 1.0, kTol);
}

TEST(ProjectOntoPath, MultiSegmentPicksNearest) {
  // Three-waypoint path (0,0)->(1,0)->(2,0). Robot at (1.6, 0.3).
  // Nearest segment is the second one [1,0]->[2,0].
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0), makeXY(2.0, 0.0) };
  PathProjection p = projectOntoPath(path, 1.6, 0.3);

  EXPECT_TRUE(p.valid);
  EXPECT_EQ(p.seg, static_cast<size_t>(1));       // second segment
  EXPECT_NEAR(p.point.x, 1.6, kTol);
  EXPECT_NEAR(p.point.y, 0.0, kTol);
  EXPECT_NEAR(p.arc_length, 1.6, kTol);           // 1 (first seg) + 0.6
}

// ---------------------------------------------------------------------------
// pointAtArcLength

TEST(PointAtArcLength, SZeroReturnsFirstPoint) {
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  double tan_yaw = -99.0;
  Pose2D p = pointAtArcLength(path, 0.0, tan_yaw);

  EXPECT_NEAR(p.x,    0.0, kTol);
  EXPECT_NEAR(p.y,    0.0, kTol);
  EXPECT_NEAR(tan_yaw, 0.0, kTol);  // +x tangent
}

TEST(PointAtArcLength, STotalReturnsLastPoint) {
  const double L = 2.5;
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(L, 0.0) };
  double tan_yaw = -99.0;
  Pose2D p = pointAtArcLength(path, L, tan_yaw);

  EXPECT_NEAR(p.x,    L,   kTol);
  EXPECT_NEAR(p.y,    0.0, kTol);
  EXPECT_NEAR(tan_yaw, 0.0, kTol);
}

TEST(PointAtArcLength, SBeyondTotalClampsToLastPoint) {
  const double L = 1.0;
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(L, 0.0) };
  double tan_yaw = -99.0;
  Pose2D p = pointAtArcLength(path, L + 5.0, tan_yaw);

  EXPECT_NEAR(p.x, L,   kTol);
  EXPECT_NEAR(p.y, 0.0, kTol);
}

TEST(PointAtArcLength, SNegativeClampsToFirstPoint) {
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  double tan_yaw = -99.0;
  Pose2D p = pointAtArcLength(path, -1.0, tan_yaw);

  EXPECT_NEAR(p.x, 0.0, kTol);
  EXPECT_NEAR(p.y, 0.0, kTol);
}

TEST(PointAtArcLength, SMidpointInterpolated) {
  // Path (0,0)->(2,0), s=1 -> (1,0).
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(2.0, 0.0) };
  double tan_yaw = -99.0;
  Pose2D p = pointAtArcLength(path, 1.0, tan_yaw);

  EXPECT_NEAR(p.x,    1.0, kTol);
  EXPECT_NEAR(p.y,    0.0, kTol);
  EXPECT_NEAR(tan_yaw, 0.0, kTol);
}

TEST(PointAtArcLength, TangentYawOnDiagonalSegment) {
  // Path along y=x: (0,0)->(1,1). Tangent should be pi/4.
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 1.0) };
  double tan_yaw = -99.0;
  Pose2D p = pointAtArcLength(path, 0.5, tan_yaw);

  EXPECT_NEAR(tan_yaw, M_PI / 4.0, kTol);
  // point should be at (0.5/sqrt(2) ... but more simply at the midpoint)
  const double half = std::sqrt(2.0) / 2.0 * 0.5;
  EXPECT_NEAR(p.x, half, kTol);
  EXPECT_NEAR(p.y, half, kTol);
}

TEST(PointAtArcLength, MultiSegmentPathCrossesJunction) {
  // Path (0,0)->(1,0)->(1,1). Total length = 2. s=1.5 is 0.5 into second segment.
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0), makeXY(1.0, 1.0) };
  double tan_yaw = -99.0;
  Pose2D p = pointAtArcLength(path, 1.5, tan_yaw);

  EXPECT_NEAR(p.x, 1.0, kTol);
  EXPECT_NEAR(p.y, 0.5, kTol);
  EXPECT_NEAR(tan_yaw, M_PI / 2.0, kTol);  // second segment is along +y

  // M2: boundary-ownership convention -- at s exactly equal to the junction arc
  // length (s=1.0, the shared vertex between first and second segments) the
  // implementation selects the FIRST (incoming) segment because the loop exits as
  // soon as s <= arc_end. Tangent must therefore report ~0 (along +x), not pi/2.
  double tan_at_junction = -99.0;
  Pose2D p_junc = pointAtArcLength(path, 1.0, tan_at_junction);
  EXPECT_NEAR(p_junc.x, 1.0, kTol);
  EXPECT_NEAR(p_junc.y, 0.0, kTol);
  EXPECT_NEAR(tan_at_junction, 0.0, kTol) << "at s==junction the incoming segment tangent is used";
}

TEST(PointAtArcLength, SinglePointPath) {
  std::vector<Pose2D> path = { makeXY(7.0, 3.0) };
  double tan_yaw = -99.0;
  Pose2D p = pointAtArcLength(path, 0.0, tan_yaw);

  EXPECT_NEAR(p.x,    7.0, kTol);
  EXPECT_NEAR(p.y,    3.0, kTol);
  EXPECT_NEAR(tan_yaw, 0.0, kTol);
}

// ---------------------------------------------------------------------------
// generateReference

// Default MpcConfig with horizon=5, dt=0.1, v_ref=0.5, goal_align_radius=0.5.
static MpcConfig defaultCfg() {
  MpcConfig cfg;
  cfg.horizon           = 5;
  cfg.dt                = 0.1;
  cfg.v_ref             = 0.5;
  cfg.goal_align_radius = 0.5;
  return cfg;
}

TEST(GenerateReference, EmptyPathReturnsEmpty) {
  std::vector<Pose2D> path;
  Pose2D robot, goal;
  auto refs = generateReference(path, robot, goal, defaultCfg());
  EXPECT_TRUE(refs.empty());
}

TEST(GenerateReference, SizeEqualsHorizon) {
  const double L = 10.0;
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(L, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  Pose2D goal  = makeXY(L,   0.0);

  MpcConfig cfg = defaultCfg();
  auto refs = generateReference(path, robot, goal, cfg);

  EXPECT_EQ(static_cast<int>(refs.size()), cfg.horizon);
}

TEST(GenerateReference, StraightPathSpacingAndTangent) {
  // Path (0,0)->(10,0). Robot at origin. Far from goal (remaining >> goal_align_radius).
  const double L = 10.0;
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(L, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  Pose2D goal;
  goal.x   = L;
  goal.y   = 0.0;
  goal.yaw = 1.5;  // arbitrary; should NOT appear since we are far from goal

  MpcConfig cfg = defaultCfg();
  cfg.horizon = 5;
  cfg.v_ref   = 0.5;
  cfg.dt      = 0.1;

  auto refs = generateReference(path, robot, goal, cfg);

  ASSERT_EQ(static_cast<int>(refs.size()), cfg.horizon);
  for (int k = 0; k < cfg.horizon; ++k) {
    const double expected_x = cfg.v_ref * cfg.dt * static_cast<double>(k + 1);
    EXPECT_NEAR(refs[static_cast<size_t>(k)].x,   expected_x, kTol) << "k=" << k;
    EXPECT_NEAR(refs[static_cast<size_t>(k)].y,   0.0,        kTol) << "k=" << k;
    EXPECT_NEAR(refs[static_cast<size_t>(k)].yaw, 0.0,        kTol) << "k=" << k;  // tangent along +x
  }

  // Consecutive spacing should be v_ref*dt.
  const double step = cfg.v_ref * cfg.dt;
  for (int k = 1; k < cfg.horizon; ++k) {
    const double dx = refs[static_cast<size_t>(k)].x - refs[static_cast<size_t>(k - 1)].x;
    EXPECT_NEAR(dx, step, kTol) << "k=" << k;
  }
}

TEST(GenerateReference, CornerPathYawTransition) {
  // Path (0,0)->(1,0)->(1,1): tangent is 0 on first segment, pi/2 on second.
  // Robot at origin, far from goal. References should transition from yaw~0 to yaw~pi/2.
  // Use v_ref=1.0, dt=0.1, horizon=20 so the step arc s_k = 0.1*k.
  // At k=10, s_10=1.0 (segment junction); at k=11, s_11=1.1 (second segment).
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0), makeXY(1.0, 1.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  Pose2D goal;
  goal.x   = 1.0;
  goal.y   = 1.0;
  goal.yaw = 0.0;

  MpcConfig cfg = defaultCfg();
  cfg.horizon           = 20;
  cfg.v_ref             = 1.0;   // 1 m/s so step=0.1 m; k=11 -> s=1.1 m on second segment
  cfg.dt                = 0.1;
  cfg.goal_align_radius = 0.05;  // tiny so we stay in travel-tangent mode longer
  cfg.a_decel           = 1e9;   // neutralize the approach ramp: keep constant-v_ref spacing

  auto refs = generateReference(path, robot, goal, cfg);
  ASSERT_EQ(static_cast<int>(refs.size()), 20);

  // Step k=0 (k=1 in 1-indexed): s_1=0.1, still on first segment -> yaw ~ 0.
  EXPECT_NEAR(refs[0].yaw, 0.0, kTol);

  // Find the first reference strictly past the junction at s=1.0 and check pi/2.
  // With v_ref=1.0 and dt=0.1, s_k = 0.1*k. k=11 gives s=1.1.
  bool found_second = false;
  for (int k = 0; k < 20; ++k) {
    const double s_k = cfg.v_ref * cfg.dt * static_cast<double>(k + 1);
    if (s_k > 1.0 && s_k <= 2.0) {
      // This ref should be on the second segment (along +y from (1,0) to (1,1)).
      EXPECT_NEAR(refs[static_cast<size_t>(k)].yaw, M_PI / 2.0, kTol) << "k=" << k;
      EXPECT_NEAR(refs[static_cast<size_t>(k)].x, 1.0, kTol) << "k=" << k;
      found_second = true;
      break;
    }
  }
  EXPECT_TRUE(found_second) << "No reference found on the second segment";
}

TEST(GenerateReference, GoalAlignHeadingSwitchAndEndClamp) {
  // Short path (0,0)->(0.3,0). Total length L = 0.3.
  // Robot at (0.0, 0.0). remaining = 0.3, goal_align_radius = 0.5 -> goal_align = true.
  // horizon=12, v_ref=0.5, dt=0.1: s_k = 0.05*k, so s_7=0.35>L, s_12=0.60>L.
  // The later refs must clamp at x=L; this exercises the min(..., total_len) saturation.
  const double L = 0.3;
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(L, 0.0) };

  Pose2D robot = makeXY(0.0, 0.0);
  Pose2D goal;
  goal.x   = L;
  goal.y   = 0.0;
  goal.yaw = 1.2;  // goal yaw different from path tangent (0)

  MpcConfig cfg = defaultCfg();
  cfg.horizon           = 12;   // raised so later s_k exceed L=0.3
  cfg.v_ref             = 0.5;
  cfg.dt                = 0.1;
  cfg.goal_align_radius = 0.5;  // larger than the total path length -> goal_align true

  auto refs = generateReference(path, robot, goal, cfg);
  ASSERT_EQ(static_cast<int>(refs.size()), 12);

  for (size_t k = 0; k < refs.size(); ++k) {
    // yaw must equal goal.yaw (goal-align active).
    EXPECT_NEAR(refs[k].yaw, goal.yaw, kTol) << "k=" << k;
    // x must not exceed L (end clamp).
    EXPECT_LE(refs[k].x, L + kTol) << "k=" << k;
  }

  // The final ref (k=11, s_12 = 0.5*0.1*12 = 0.60 > L) must be clamped exactly at L.
  // This confirms the min(proj.arc_length + v_ref*dt*k, total_len) branch is hit.
  EXPECT_NEAR(refs[11].x, L, kTol) << "last ref must clamp to path end";
  EXPECT_NEAR(refs[11].y, 0.0, kTol);
  EXPECT_NEAR(refs[11].yaw, goal.yaw, kTol);
}

TEST(GenerateReference, RobotPastPathEndClampsAllRefs) {
  // Robot is past the path end. Projection should clamp to the last point.
  // remaining ~ 0 -> goal_align active; all refs at path end with goal.yaw.
  const double L = 1.0;
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(L, 0.0) };

  Pose2D robot = makeXY(5.0, 0.0);  // well past the end
  Pose2D goal;
  goal.x   = L;
  goal.y   = 0.0;
  goal.yaw = 0.7;

  MpcConfig cfg = defaultCfg();
  cfg.horizon           = 5;
  cfg.goal_align_radius = 0.5;  // remaining ~ 0 < radius -> goal align

  auto refs = generateReference(path, robot, goal, cfg);
  ASSERT_EQ(static_cast<int>(refs.size()), 5);

  for (size_t k = 0; k < refs.size(); ++k) {
    EXPECT_NEAR(refs[k].x,   L,        kTol) << "k=" << k;
    EXPECT_NEAR(refs[k].y,   0.0,      kTol) << "k=" << k;
    EXPECT_NEAR(refs[k].yaw, goal.yaw, kTol) << "k=" << k;
  }
}

TEST(GenerateReference, TravelTangentYawWhenFarFromGoal) {
  // Very long path; robot near start. All refs should have tangent yaw, not goal.yaw.
  const double L = 100.0;
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(L, 0.0) };

  Pose2D robot = makeXY(0.0, 0.0);
  Pose2D goal;
  goal.x   = L;
  goal.y   = 0.0;
  goal.yaw = 3.14;  // clearly different from tangent (0)

  MpcConfig cfg = defaultCfg();
  cfg.horizon           = 5;
  cfg.goal_align_radius = 0.5;  // much less than remaining (~100 m)

  auto refs = generateReference(path, robot, goal, cfg);
  ASSERT_EQ(static_cast<int>(refs.size()), 5);

  for (size_t k = 0; k < refs.size(); ++k) {
    // Should be tangent yaw (0), not goal.yaw (3.14).
    EXPECT_NEAR(refs[k].yaw, 0.0, kTol) << "k=" << k;
  }
}

TEST(GenerateReference, TerminalRefRetargetsToRawGoalWhenReachable) {
  // Path (0,0)->(0.3,0); raw goal (0.4,0): endpoint-to-goal gap 0.1 <= reach_goal_gap.
  // In the goal-align zone the clamped terminal refs must sit at the RAW goal (0.4),
  // not the path endpoint (0.3).
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(0.3, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  Pose2D goal; goal.x = 0.4; goal.y = 0.0; goal.yaw = 0.0;

  MpcConfig cfg = defaultCfg();
  cfg.horizon = 12;                      // later s_k exceed the 0.3 path length -> clamped
  cfg.goal_align_radius = 0.5;           // remaining 0.3 < radius -> goal-align active
  cfg.reach_goal_gap = 0.15;

  auto refs = generateReference(path, robot, goal, cfg);
  ASSERT_EQ(static_cast<int>(refs.size()), 12);
  EXPECT_NEAR(refs[11].x, 0.4, kTol) << "terminal ref must retarget to the raw goal";
  EXPECT_NEAR(refs[11].y, 0.0, kTol);
}

TEST(GenerateReference, TerminalRefClampsToPathEndWhenUnreachable) {
  // Same path; raw goal (1.0,0): gap 0.7 > reach_goal_gap -> NOT reachable.
  // Terminal refs must clamp at the path endpoint (0.3), the existing behavior.
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(0.3, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  Pose2D goal; goal.x = 1.0; goal.y = 0.0; goal.yaw = 0.0;

  MpcConfig cfg = defaultCfg();
  cfg.horizon = 12;
  cfg.goal_align_radius = 0.5;
  cfg.reach_goal_gap = 0.15;

  auto refs = generateReference(path, robot, goal, cfg);
  ASSERT_EQ(static_cast<int>(refs.size()), 12);
  EXPECT_NEAR(refs[11].x, 0.3, kTol) << "terminal ref must clamp at the path end";
  EXPECT_NEAR(refs[11].y, 0.0, kTol);
}

// ---------------------------------------------------------------------------
// M3: degenerate paths with zero-length segments

TEST(ProjectOntoPath, ZeroLengthSegmentInsidePath) {
  // Path (0,0)->(0,0)->(1,0): first segment is zero-length (duplicate waypoints).
  // projectOntoPath must not crash or produce NaN. Total arc length = 1.0.
  // A robot at (0.5, 0.0) should project to (0.5, 0.0) on the real segment.
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  PathProjection p = projectOntoPath(path, 0.5, 0.0);

  EXPECT_TRUE(p.valid);
  EXPECT_NEAR(p.point.x, 0.5, kTol);
  EXPECT_NEAR(p.point.y, 0.0, kTol);
  // arc_length from path[0]: zero-length seg contributes 0, then 0.5 into the real seg.
  EXPECT_NEAR(p.arc_length, 0.5, kTol);
  EXPECT_FALSE(std::isnan(p.arc_length));
  EXPECT_FALSE(std::isnan(p.tangent_yaw));
}

TEST(PointAtArcLength, ZeroLengthSegmentInsidePath) {
  // Path (0,0)->(0,0)->(1,0): total length = 1.0.
  // s=0.5 should interpolate on the real segment and return (0.5, 0.0).
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  double tan_yaw = -99.0;
  Pose2D p = pointAtArcLength(path, 0.5, tan_yaw);

  EXPECT_NEAR(p.x, 0.5, kTol);
  EXPECT_NEAR(p.y, 0.0, kTol);
  EXPECT_FALSE(std::isnan(p.x));
  EXPECT_FALSE(std::isnan(p.y));
  EXPECT_FALSE(std::isnan(tan_yaw));
}

TEST(GenerateReference, ZeroLengthSegmentInsidePathSane) {
  // Path (0,0)->(0,0)->(1,0): duplicate first waypoint. Robot at origin.
  // generateReference must not crash; point at s=0.5 should be near (0.5, 0.0).
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  Pose2D goal;
  goal.x   = 1.0;
  goal.y   = 0.0;
  goal.yaw = 0.0;

  MpcConfig cfg = defaultCfg();
  cfg.horizon           = 5;
  cfg.v_ref             = 1.0;  // s_k = 0.1*k; s_5 = 0.5
  cfg.dt                = 0.1;
  cfg.goal_align_radius = 0.05;  // far from goal, travel tangent mode
  cfg.a_decel    = 1e9;  // neutralize the approach ramp: keep constant-v_ref spacing
  cfg.a_lat_max  = 1e9;  // neutralize the curvature regulator: keep constant-v_ref spacing

  auto refs = generateReference(path, robot, goal, cfg);
  ASSERT_EQ(static_cast<int>(refs.size()), 5);

  for (size_t k = 0; k < refs.size(); ++k) {
    EXPECT_FALSE(std::isnan(refs[k].x))   << "k=" << k;
    EXPECT_FALSE(std::isnan(refs[k].y))   << "k=" << k;
    EXPECT_FALSE(std::isnan(refs[k].yaw)) << "k=" << k;
  }

  // s_5 = 0.5 should place the last ref at x=0.5.
  EXPECT_NEAR(refs[4].x, 0.5, kTol);
  EXPECT_NEAR(refs[4].y, 0.0, kTol);
}

TEST(GenerateReference, FullyDegeneratePathAllIdentical) {
  // Path (0,0)->(0,0)->(0,0): all identical waypoints, total_len = 0.
  // generateReference must return horizon refs all at (0,0) with goal.yaw
  // (goal_align is true because remaining=0 <= any positive goal_align_radius).
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(0.0, 0.0), makeXY(0.0, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  Pose2D goal;
  goal.x   = 0.0;
  goal.y   = 0.0;
  goal.yaw = 2.5;

  MpcConfig cfg = defaultCfg();
  cfg.horizon           = 5;
  cfg.v_ref             = 0.5;
  cfg.dt                = 0.1;
  cfg.goal_align_radius = 0.5;

  auto refs = generateReference(path, robot, goal, cfg);
  ASSERT_EQ(static_cast<int>(refs.size()), 5);

  for (size_t k = 0; k < refs.size(); ++k) {
    EXPECT_NEAR(refs[k].x,   0.0,      kTol) << "k=" << k;
    EXPECT_NEAR(refs[k].y,   0.0,      kTol) << "k=" << k;
    EXPECT_NEAR(refs[k].yaw, goal.yaw, kTol) << "k=" << k;
  }
}

// ---------------------------------------------------------------------------
// referenceSpeed: approach decel-to-stop ramp

// Helper: total polyline arc length (the file-static one is not exported).
static double arcLenOf(const std::vector<Pose2D>& path) {
  double t = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i)
    t += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  return t;
}

TEST(ReferenceSpeed, FarFromGoalCruisesAtVref) {
  // Long straight path, robot at start, heading aligned: nothing should slow it.
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(10.0, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);  // yaw 0, tangent 0 -> no heading slowdown
  MpcConfig cfg = defaultCfg();
  const double L = arcLenOf(path);
  const auto proj = projectOntoPath(path, robot.x, robot.y);
  const double v = g1_local_planner::referenceSpeed(path, proj, L, robot, cfg);
  EXPECT_NEAR(v, cfg.v_ref, kTol);
}

TEST(ReferenceSpeed, ApproachRampBindsNearGoal) {
  // Straight 1 m path; robot 0.2 m from the end. v_approach = sqrt(2*a_decel*0.2).
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  Pose2D robot = makeXY(0.8, 0.0);
  MpcConfig cfg = defaultCfg();   // a_decel = 0.2 (struct default)
  const double L = arcLenOf(path);
  const auto proj = projectOntoPath(path, robot.x, robot.y);
  const double v = g1_local_planner::referenceSpeed(path, proj, L, robot, cfg);
  const double expected = std::sqrt(2.0 * cfg.a_decel * 0.2);  // ~0.28
  EXPECT_NEAR(v, expected, 1e-6);
  EXPECT_LT(v, cfg.v_ref);  // it actually slowed down
}

TEST(ReferenceSpeed, ZeroAtGoal) {
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  Pose2D robot = makeXY(1.0, 0.0);  // at the end, remaining = 0
  MpcConfig cfg = defaultCfg();
  const double L = arcLenOf(path);
  const auto proj = projectOntoPath(path, robot.x, robot.y);
  const double v = g1_local_planner::referenceSpeed(path, proj, L, robot, cfg);
  EXPECT_NEAR(v, 0.0, kTol);
}

TEST(ReferenceSpeed, ApproachMonotoneNonIncreasingIntoGoal) {
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(1.0, 0.0) };
  MpcConfig cfg = defaultCfg();
  const double L = arcLenOf(path);
  double prev = 1e9;
  for (double x = 0.6; x <= 1.0 + 1e-9; x += 0.05) {
    Pose2D robot = makeXY(x, 0.0);
    const auto proj = projectOntoPath(path, robot.x, robot.y);
    const double v = g1_local_planner::referenceSpeed(path, proj, L, robot, cfg);
    EXPECT_LE(v, prev + 1e-9) << "x=" << x;
    prev = v;
  }
}

// ---------------------------------------------------------------------------
// referenceSpeed: curvature regulator

// Helper: quarter-circle polyline of radius R, starting at origin heading +x,
// curving CCW (center at (0, R)). n segments. Tangent angle at arc s is s/R.
static std::vector<Pose2D> quarterArc(double R, int n) {
  std::vector<Pose2D> path;
  path.reserve(static_cast<size_t>(n + 1));
  for (int i = 0; i <= n; ++i) {
    const double a = (M_PI / 2.0) * static_cast<double>(i) / static_cast<double>(n);
    path.push_back(makeXY(R * std::sin(a), R - R * std::cos(a)));
  }
  return path;
}

TEST(ReferenceSpeed, StraightPathNoCurvatureSlowdown) {
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(10.0, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  MpcConfig cfg = defaultCfg();
  const double L = arcLenOf(path);
  const auto proj = projectOntoPath(path, robot.x, robot.y);
  EXPECT_NEAR(g1_local_planner::referenceSpeed(path, proj, L, robot, cfg),
              cfg.v_ref, kTol);
}

TEST(ReferenceSpeed, TightArcSlowsToLateralAccelLimit) {
  // R=0.3 -> kappa=1/R=3.333. v_curv = sqrt(a_lat_max/kappa) = sqrt(0.5*0.3) ~ 0.387.
  const double R = 0.3;
  std::vector<Pose2D> path = quarterArc(R, 60);
  Pose2D robot = makeXY(0.0, 0.0);  // at the arc start, tangent +x, yaw 0
  MpcConfig cfg = defaultCfg();     // a_lat_max=0.5
  const double L = arcLenOf(path);
  const auto proj = projectOntoPath(path, robot.x, robot.y);
  const double v = g1_local_planner::referenceSpeed(path, proj, L, robot, cfg);
  const double expected = std::sqrt(cfg.a_lat_max * R);  // sqrt(a_lat_max/(1/R))
  EXPECT_LT(v, cfg.v_ref);                 // it slowed for the bend
  EXPECT_NEAR(v, expected, 0.06);          // discrete-curvature tolerance
}

TEST(ReferenceSpeed, LargeLatAccelDisablesCurvatureSlowdown) {
  const double R = 0.3;
  std::vector<Pose2D> path = quarterArc(R, 60);
  Pose2D robot = makeXY(0.0, 0.0);
  MpcConfig cfg = defaultCfg();
  cfg.a_lat_max = 1e9;  // effectively no lateral-accel limit
  cfg.a_decel   = 1e9;  // neutralize the approach ramp so only the curvature regulator is tested
  const double L = arcLenOf(path);
  const auto proj = projectOntoPath(path, robot.x, robot.y);
  EXPECT_NEAR(g1_local_planner::referenceSpeed(path, proj, L, robot, cfg),
              cfg.v_ref, kTol);
}

// ---------------------------------------------------------------------------
// referenceSpeed: heading-error regulator

TEST(ReferenceSpeed, HeadingAlignedNoSlowdown) {
  // Robot heading error 0.2 rad < heading_slow_start (0.3): no slowdown.
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(10.0, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  robot.yaw = 0.2;
  MpcConfig cfg = defaultCfg();
  const double L = arcLenOf(path);
  const auto proj = projectOntoPath(path, robot.x, robot.y);
  EXPECT_NEAR(g1_local_planner::referenceSpeed(path, proj, L, robot, cfg),
              cfg.v_ref, kTol);
}

TEST(ReferenceSpeed, HeadingMidErrorScalesSpeed) {
  // theta_err = 0.5: factor = (1.0-0.5)/(1.0-0.3) = 0.7142857; v = 0.5*that.
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(10.0, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  robot.yaw = 0.5;
  MpcConfig cfg = defaultCfg();
  const double L = arcLenOf(path);
  const auto proj = projectOntoPath(path, robot.x, robot.y);
  const double factor = (cfg.heading_slow_full - 0.5)
                      / (cfg.heading_slow_full - cfg.heading_slow_start);
  EXPECT_NEAR(g1_local_planner::referenceSpeed(path, proj, L, robot, cfg),
              cfg.v_ref * factor, 1e-6);
}

TEST(ReferenceSpeed, HeadingLargeErrorHitsFloorThenVminMove) {
  // theta_err = 1.2 > heading_slow_full (1.0): factor = floor (0) -> v_heading 0,
  // so the cruise min is the v_min_move floor (0.05).
  std::vector<Pose2D> path = { makeXY(0.0, 0.0), makeXY(10.0, 0.0) };
  Pose2D robot = makeXY(0.0, 0.0);
  robot.yaw = 1.2;
  MpcConfig cfg = defaultCfg();
  const double L = arcLenOf(path);
  const auto proj = projectOntoPath(path, robot.x, robot.y);
  EXPECT_NEAR(g1_local_planner::referenceSpeed(path, proj, L, robot, cfg),
              cfg.v_min_move, kTol);
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
