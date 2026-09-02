// Unit tests for the LocalPlanner facade: reference + MPC + terminal /
// stop logic, plus the warm-start obstacle nominal persisting across cycles and a
// short closed-loop end-to-end run.
//
// Tests are MEANINGFUL: precondition gates (no goal / no costmap / no path) each
// yield zero command + the matching state; GOAL_REACHED stops at the goal; a free
// corridor tracks forward; the warm-start nominal persists and sharpens avoidance
// between calls; and a short closed loop reaches the goal collision-free. OSQP is
// iterative so tolerances are loose.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "g1_local_planner/core/costmap_view.h"
#include "g1_local_planner/core/kinematic_model.h"
#include "g1_local_planner/core/plan_types.h"
#include "g1_local_planner/planner/local_planner.h"

using g1_local_planner::ControllerState;
using g1_local_planner::CostmapView;
using g1_local_planner::LocalPlanner;
using g1_local_planner::MpcConfig;
using g1_local_planner::MpcResult;
using g1_local_planner::Pose2D;
using g1_local_planner::stepTrue;
using g1_local_planner::Twist2D;

namespace {

// A uniform-free costmap covering half_m metres each side of (cx, cy) at 0.1 m.
CostmapView freeCostmap(double cx, double cy, double half_m,
                        const MpcConfig& cfg) {
  const double res = 0.1;
  const int n = static_cast<int>(std::round(2.0 * half_m / res));
  const double ox = cx - half_m;
  const double oy = cy - half_m;
  std::vector<int8_t> data(static_cast<size_t>(n) * static_cast<size_t>(n), 0);
  return CostmapView(res, ox, oy, n, n, data, cfg.lethal_cost,
                     cfg.treat_unknown_as_obstacle);
}

// A straight path along +x from the origin to (length, 0), sampled every step m.
std::vector<Pose2D> straightPathX(double length, double step) {
  std::vector<Pose2D> path;
  for (double s = 0.0; s <= length + 1e-9; s += step) {
    Pose2D p;
    p.x = s;
    p.y = 0.0;
    p.yaw = 0.0;
    path.push_back(p);
  }
  return path;
}

// A uniform-free costmap with a lethal column at every cell whose world x >= wall_x.
CostmapView corridorWithWall(double cx, double cy, double half_m, double wall_x,
                             const MpcConfig& cfg) {
  const double res = 0.1;
  const int n = static_cast<int>(std::round(2.0 * half_m / res));
  const double ox = cx - half_m;
  const double oy = cy - half_m;
  std::vector<int8_t> data(static_cast<size_t>(n) * static_cast<size_t>(n), 0);
  for (int my = 0; my < n; ++my) {
    for (int mx = 0; mx < n; ++mx) {
      const double wx = ox + (static_cast<double>(mx) + 0.5) * res;
      if (wx >= wall_x) data[static_cast<size_t>(my) * n + mx] = 100;
    }
  }
  return CostmapView(res, ox, oy, n, n, data, cfg.lethal_cost,
                     cfg.treat_unknown_as_obstacle);
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. No goal -> NO_GOAL, zero command.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, NoGoal) {
  MpcConfig cfg;
  LocalPlanner lp(cfg);
  lp.setCostmap(freeCostmap(0.0, 0.0, 3.0, cfg));
  lp.setPath(straightPathX(2.0, 0.1));
  // no setGoal

  const MpcResult r = lp.compute(Pose2D{});
  EXPECT_EQ(r.status, ControllerState::NO_GOAL);
  EXPECT_DOUBLE_EQ(r.applied.vx, 0.0);
  EXPECT_DOUBLE_EQ(r.applied.vy, 0.0);
  EXPECT_DOUBLE_EQ(r.applied.w, 0.0);
}

// ---------------------------------------------------------------------------
// 2. No costmap -> NO_COSTMAP, zero command.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, NoCostmap) {
  MpcConfig cfg;
  LocalPlanner lp(cfg);
  lp.setPath(straightPathX(2.0, 0.1));
  Pose2D goal;
  goal.x = 2.0;
  lp.setGoal(goal);
  // no setCostmap -> default invalid CostmapView

  const MpcResult r = lp.compute(Pose2D{});
  EXPECT_EQ(r.status, ControllerState::NO_COSTMAP);
  EXPECT_DOUBLE_EQ(r.applied.vx, 0.0);
  EXPECT_DOUBLE_EQ(r.applied.vy, 0.0);
  EXPECT_DOUBLE_EQ(r.applied.w, 0.0);
}

// ---------------------------------------------------------------------------
// 3. Empty path -> NO_PATH, zero command.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, EmptyPath) {
  MpcConfig cfg;
  LocalPlanner lp(cfg);
  lp.setCostmap(freeCostmap(0.0, 0.0, 3.0, cfg));
  Pose2D goal;
  goal.x = 2.0;
  lp.setGoal(goal);
  // path left empty

  const MpcResult r = lp.compute(Pose2D{});
  EXPECT_EQ(r.status, ControllerState::NO_PATH);
  EXPECT_DOUBLE_EQ(r.applied.vx, 0.0);
  EXPECT_DOUBLE_EQ(r.applied.vy, 0.0);
  EXPECT_DOUBLE_EQ(r.applied.w, 0.0);
}

// ---------------------------------------------------------------------------
// 4. Goal reached: current within tol of the goal -> GOAL_REACHED, zero command.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, GoalReached) {
  MpcConfig cfg;
  LocalPlanner lp(cfg);
  lp.setCostmap(freeCostmap(1.0, 0.0, 3.0, cfg));
  lp.setPath(straightPathX(2.0, 0.1));  // path ends at (2.0, 0) = the goal

  Pose2D goal;
  goal.x = 2.0;  // goal coincides with the path endpoint (the realistic case)
  goal.y = 0.0;
  goal.yaw = 0.0;
  lp.setGoal(goal);

  // current coincides with the goal / path end (well within xy_tol / yaw_tol).
  Pose2D cur;
  cur.x = 2.0;
  const MpcResult r = lp.compute(cur);
  EXPECT_EQ(r.status, ControllerState::GOAL_REACHED);
  EXPECT_DOUBLE_EQ(r.applied.vx, 0.0);
  EXPECT_DOUBLE_EQ(r.applied.vy, 0.0);
  EXPECT_DOUBLE_EQ(r.applied.w, 0.0);
}

// ---------------------------------------------------------------------------
// Default goal tolerances are the tightened precision targets.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, DefaultGoalTolerancesAreTight) {
  MpcConfig cfg;
  EXPECT_DOUBLE_EQ(cfg.xy_goal_tolerance, 0.05);
  EXPECT_DOUBLE_EQ(cfg.yaw_goal_tolerance, 0.03);
}

// ---------------------------------------------------------------------------
// 4b. clearGoal after a reached goal -> NO_GOAL, not a stale GOAL_REACHED.
//     The node calls clearGoal() when it is disabled; this is the invariant
//     that keeps an idle/disabled controller from latching the just-finished
//     goal's GOAL_REACHED (which the coordinator would read on the first tick
//     after a NEW goal and spuriously finish -- a far goal HALTing un-moved).
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, ClearGoalDropsStaleReached) {
  MpcConfig cfg;
  LocalPlanner lp(cfg);
  lp.setCostmap(freeCostmap(1.0, 0.0, 3.0, cfg));
  lp.setPath(straightPathX(2.0, 0.1));  // path ends at (2.0, 0) = the goal

  Pose2D goal;
  goal.x = 2.0;  // goal at the path end; current coincides -> reached
  lp.setGoal(goal);
  Pose2D cur;
  cur.x = 2.0;
  EXPECT_EQ(lp.compute(cur).status, ControllerState::GOAL_REACHED);

  // Disabling clears the goal: with the costmap + path still set, compute must
  // now report NO_GOAL (precedence: goal is checked before reached), not the
  // stale GOAL_REACHED.
  lp.clearGoal();
  EXPECT_EQ(lp.compute(cur).status, ControllerState::NO_GOAL);
}

// ---------------------------------------------------------------------------
// 5. Tracking: free costmap + straight path + a far goal -> TRACKING, vx > 0,
//    predicted size == horizon.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, Tracking) {
  MpcConfig cfg;
  LocalPlanner lp(cfg);
  lp.setCostmap(freeCostmap(1.0, 0.0, 4.0, cfg));
  lp.setPath(straightPathX(4.0, 0.1));

  Pose2D goal;
  goal.x = 4.0;  // far ahead, not reached
  lp.setGoal(goal);

  const MpcResult r = lp.compute(Pose2D{});  // current at start
  EXPECT_EQ(r.status, ControllerState::TRACKING);
  EXPECT_GT(r.applied.vx, 0.0);
  EXPECT_EQ(r.predicted.size(), static_cast<size_t>(cfg.horizon));
}

// ---------------------------------------------------------------------------
// 6. Warm-start state persists: after a solved compute, last_predicted is carried
//    and used as the obstacle nominal on the next compute. We observe this two
//    ways: lastPredicted() is non-empty (size == horizon) between calls, and on a
//    head-on-ish obstacle the SECOND compute (which sees the warm-start nominal
//    the FIRST compute stored) bows further off than the first.
//    As in MpcTest.WarmStartSharpensAvoidance, the robust observable is the single
//    warm-started step (compute 2 vs compute 1); the undamped warm start limit-
//    cycles if iterated on a STATIC obstacle, which is why we compare exactly the
//    two adjacent computes rather than a many-step fixed point.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, WarmStartStatePersists) {
  MpcConfig cfg;
  LocalPlanner lp(cfg);

  // Obstacle slightly off the line (breaks perfect head-on symmetry so the first
  // cycle already sees a small lateral gradient; see the head-on caveat in mpc.h).
  const double res = 0.1;
  const double half = 3.0;
  const int n = static_cast<int>(std::round(2.0 * half / res));
  const double ox = -half;
  const double oy = -half;
  std::vector<int8_t> data(static_cast<size_t>(n) * static_cast<size_t>(n), 0);
  const double obs_cx = 0.5;
  const double obs_cy = 0.05;
  auto setCell = [&](double wx, double wy, int8_t v) {
    int mx = static_cast<int>(std::floor((wx - ox) / res));
    int my = static_cast<int>(std::floor((wy - oy) / res));
    if (mx >= 0 && mx < n && my >= 0 && my < n) {
      int8_t& cell = data[static_cast<size_t>(my) * n + mx];
      if (v > cell) cell = v;
    }
  };
  for (double wx = -0.1; wx <= 1.1; wx += res * 0.5) {
    for (double wy = -0.7; wy <= 0.8; wy += res * 0.5) {
      const double d = std::hypot(wx - obs_cx, wy - obs_cy);
      if (d <= 0.18) {
        setCell(wx, wy, 100);
      } else if (d <= 0.55) {
        setCell(wx, wy,
                static_cast<int8_t>(std::max(
                    1.0, std::round(99.0 * (1.0 - (d - 0.18) / (0.55 - 0.18))))));
      }
    }
  }
  lp.setCostmap(CostmapView(res, ox, oy, n, n, data, cfg.lethal_cost,
                            cfg.treat_unknown_as_obstacle));
  lp.setPath(straightPathX(2.0, 0.1));
  Pose2D goal;
  goal.x = 2.0;
  lp.setGoal(goal);

  auto maxLat = [](const std::vector<Pose2D>& pred) {
    double m = 0.0;
    for (const Pose2D& p : pred) m = std::max(m, std::abs(p.y));
    return m;
  };

  // Before any compute, no warm-start nominal is held.
  EXPECT_TRUE(lp.lastPredicted().empty());

  // First compute: stateless obstacle nominal (no warm start yet). The facade
  // then STORES this predicted path as the warm-start nominal for the next cycle.
  const MpcResult r1 = lp.compute(Pose2D{});
  ASSERT_EQ(r1.status, ControllerState::TRACKING);
  EXPECT_EQ(lp.lastPredicted().size(), static_cast<size_t>(cfg.horizon));
  const double lat1 = maxLat(r1.predicted);

  // Second compute on the SAME inputs: the facade feeds last_predicted (from
  // compute 1) back as the obstacle nominal, so this solve re-linearises about the
  // bowed path and pushes STRICTLY further off the obstacle. The stored nominal
  // also persists (still size == horizon) for the cycle after this.
  const MpcResult r2 = lp.compute(Pose2D{});
  ASSERT_EQ(r2.status, ControllerState::TRACKING);
  EXPECT_EQ(lp.lastPredicted().size(), static_cast<size_t>(cfg.horizon));
  const double lat2 = maxLat(r2.predicted);

  EXPECT_GT(lat2, lat1 + 1e-3)
      << "warm-start nominal did not persist / sharpen between computes";
}

// ---------------------------------------------------------------------------
// 7. Closed loop reaches the goal: integrate stepTrue at dt around the facade on
//    a straight free corridor; assert GOAL_REACHED within ~200 iterations, final
//    xy error <= tol, and NO pose along the way landed on a lethal cell.
//    This is the facade's end-to-end proof (the saved-map version is the offline gate).
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, ClosedLoopReachesGoal) {
  MpcConfig cfg;
  LocalPlanner lp(cfg);

  // Corridor: straight path from origin to a goal ~2 m ahead, uniform-free
  // costmap big enough to cover the whole corridor plus the horizon look-ahead.
  const double goal_x = 2.0;
  lp.setPath(straightPathX(goal_x, 0.1));
  // Centre the free window on the mid corridor with generous half-width.
  lp.setCostmap(freeCostmap(goal_x * 0.5, 0.0, 4.0, cfg));

  Pose2D goal;
  goal.x = goal_x;
  goal.y = 0.0;
  goal.yaw = 0.0;
  lp.setGoal(goal);

  Pose2D pose;  // start at the origin
  const int kMaxIters = 200;
  bool reached = false;
  bool collision_free = true;
  int iters = 0;

  // We must re-evaluate the costmap query against the SAME map we handed the
  // facade; rebuild a matching view for the collision check (uniform free ->
  // cost 0 everywhere in bounds).
  const CostmapView check_cm = freeCostmap(goal_x * 0.5, 0.0, 4.0, cfg);

  for (int i = 0; i < kMaxIters; ++i) {
    ++iters;
    const MpcResult r = lp.compute(pose);
    // Collision check on the CURRENT pose (before stepping): no lethal cell.
    if (check_cm.cost(pose.x, pose.y) >= cfg.lethal_cost) {
      collision_free = false;
    }
    if (r.status == ControllerState::GOAL_REACHED) {
      reached = true;
      break;
    }
    ASSERT_EQ(r.status, ControllerState::TRACKING)
        << "unexpected non-tracking state at iter " << i;
    // Advance the true (nonlinear) plant by the applied command over dt.
    pose = stepTrue(pose, r.applied, cfg.dt);
  }

  EXPECT_TRUE(reached) << "closed loop did not reach the goal in " << kMaxIters
                       << " iterations";
  const double final_xy = std::hypot(goal.x - pose.x, goal.y - pose.y);
  EXPECT_LE(final_xy, cfg.xy_goal_tolerance)
      << "final xy error " << final_xy << " exceeds tolerance";
  EXPECT_TRUE(collision_free) << "a pose along the loop landed on a lethal cell";
  // Sanity: it actually took a non-trivial number of steps to walk 2 m.
  EXPECT_GT(iters, 1);
}

// ---------------------------------------------------------------------------
// 8. SafetyFilterStopsBeforeLethal: isolates the footprint safety filter.
//    w_obstacle=0 makes the MPC gradient-blind so the ONLY thing that can
//    reduce vx is the post-MPC safety filter.  Two planners see identical
//    inputs: one with the filter enabled, one disabled.
//
//    Warm-start phase: 5 free-corridor compute() calls ramp the slew seed to
//    near max_vx=0.50 m/s (acc_lim_x*dt=0.15 m/s per step; 5 steps -> 0.50).
//    Test step: swap in a wall at world x=0.10 m (col 12, res 0.05 m).
//      d_usable = 0.10 - 0.05 (brake_margin) = 0.05 m
//      v_safe   = sqrt(2 * acc_lim_x * d_usable) = sqrt(2*1.5*0.05) = 0.387 m/s
//    The disabled planner outputs max_vx (0.50 m/s > 0.1 -- non-vacuous).
//    The enabled filter clamps to v_safe (0.387 m/s < 0.50 m/s).
// ---------------------------------------------------------------------------
TEST(LocalPlanner, SafetyFilterStopsBeforeLethal) {
  // Costmap dimensions shared by both free and wall maps.
  const int NX = 200;
  const int NY = 40;
  const double RES = 0.05;
  const double OX = -0.5;
  const double OY = -1.0;

  // Free costmap: no lethal cells.
  std::vector<int8_t> free_data(NX * NY, 0);

  // Wall costmap: lethal column at world x = 0.10 m.
  // col = round((0.10 - OX) / RES) = round(0.60/0.05) = 12
  std::vector<int8_t> wall_data(NX * NY, 0);
  const int wall_col = static_cast<int>(std::round((0.10 - OX) / RES));  // 12
  for (int y = 0; y < NY; ++y) wall_data[y * NX + wall_col] = 100;

  // Path and goal: straight ahead along +x, goal well beyond the wall.
  std::vector<Pose2D> path;
  for (int i = 0; i <= 40; ++i) {
    Pose2D p;
    p.x = 0.1 * i;
    p.y = 0.0;
    path.push_back(p);
  }
  Pose2D goal;
  goal.x = 4.0;
  goal.y = 0.0;
  const Pose2D cur{};  // robot at origin, heading +x

  // Helper lambda: build a planner with the given safety flag and w_obstacle=0,
  // warm-start it on a FREE costmap for several steps to ramp speed near max_vx,
  // then call once more on the WALL costmap and return the applied command.
  auto run = [&](bool safety_on) -> Twist2D {
    MpcConfig cfg;
    cfg.w_obstacle = 0.0;          // gradient-blind: MPC cannot avoid on its own
    cfg.safety.enabled = safety_on;
    LocalPlanner lp(cfg);
    lp.setPath(path);
    lp.setGoal(goal);

    // Warm-start phase: free corridor, robot at origin, let slew ramp to max_vx.
    // Each iteration raises last_cmd_.vx by acc_lim_x*dt = 1.5*0.1 = 0.15 m/s.
    // Four iterations should reach 0.60 -> clamped to max_vx=0.50 m/s.
    lp.setCostmap(CostmapView(RES, OX, OY, NX, NY, free_data,
                              cfg.lethal_cost, cfg.treat_unknown_as_obstacle));
    for (int k = 0; k < 5; ++k) {
      lp.compute(cur);  // warm-start the slew (robot stays at origin)
    }

    // Test step: swap in wall costmap and observe the applied command.
    lp.setCostmap(CostmapView(RES, OX, OY, NX, NY, wall_data,
                              cfg.lethal_cost, cfg.treat_unknown_as_obstacle));
    return lp.compute(cur).applied;
  };

  const Twist2D cmd_off = run(false);  // filter disabled: MPC unobstructed
  const Twist2D cmd_on  = run(true);   // filter enabled: should clamp

  // The disabled planner (gradient-blind MPC, no filter) must drive forward
  // meaningfully -- this makes the comparison non-vacuous.
  EXPECT_GT(cmd_off.vx, 0.1)
      << "vx_off=" << cmd_off.vx
      << " -- MPC should target near max_vx after warm-start with w_obstacle=0";

  // The enabled filter must have reduced vx strictly below the disabled case.
  EXPECT_LT(cmd_on.vx, cmd_off.vx)
      << "vx_on=" << cmd_on.vx << " vx_off=" << cmd_off.vx
      << " -- safety filter did not reduce vx";
}

// ---------------------------------------------------------------------------
// 9. reach_on_path_end: position is judged against the PATH ENDPOINT, not the
//    (possibly unreachable) raw goal. Robot sitting on the path end with a raw
//    goal 0.5 m beyond -> GOAL_REACHED when true, TRACKING when false.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, ReachOnPathEndLatchesAtPathEnd) {
  MpcConfig cfg;  // reach_on_path_end defaults true
  LocalPlanner lp(cfg);
  lp.setCostmap(freeCostmap(0.75, 0.0, 3.0, cfg));
  lp.setPath(straightPathX(1.0, 0.1));   // path ends at (1.0, 0)
  Pose2D goal;
  goal.x = 1.5;                          // raw goal 0.5 m beyond the path end
  lp.setGoal(goal);

  Pose2D at_end;
  at_end.x = 1.0;                        // robot sits on the path endpoint
  EXPECT_EQ(lp.compute(at_end).status, ControllerState::GOAL_REACHED);

  // With the flag off, the raw goal (0.5 m away) governs -> not reached.
  MpcConfig off = cfg;
  off.reach_on_path_end = false;
  LocalPlanner lp_off(off);
  lp_off.setCostmap(freeCostmap(0.75, 0.0, 3.0, off));
  lp_off.setPath(straightPathX(1.0, 0.1));
  lp_off.setGoal(goal);
  EXPECT_EQ(lp_off.compute(at_end).status, ControllerState::TRACKING);
}

// ---------------------------------------------------------------------------
// 10. Stall-near-end: a wall makes the path endpoint unreachable; the footprint
//     filter holds the robot short of it INSIDE goal_align_radius. The stall
//     latch must declare GOAL_REACHED (not spin / not STUCK forever), and it must
//     be the STALL path (final xy error to the path end exceeds xy_goal_tolerance).
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, StallNearEndLatchesGoalReached) {
  MpcConfig cfg;
  cfg.safety.enabled = true;             // footprint filter must run to brake us
  cfg.reach_stall_cycles = 5;            // 0.25 s at 20 Hz keeps the loop short
  cfg.reach_stall_radius = 0.5;          // wide band: still exercise the stall mechanism
  LocalPlanner lp(cfg);

  const double wall_x = 1.0;
  lp.setPath(straightPathX(1.2, 0.1));   // endpoint (1.2, 0) is INSIDE the wall
  lp.setCostmap(corridorWithWall(0.6, 0.0, 4.0, wall_x, cfg));
  Pose2D goal;
  goal.x = 1.2;                          // unreachable raw goal
  lp.setGoal(goal);

  Pose2D pose;                           // start at the origin
  bool reached = false;
  int iters = 0;
  for (int i = 0; i < 200; ++i) {
    ++iters;
    const MpcResult r = lp.compute(pose);
    if (r.status == ControllerState::GOAL_REACHED) { reached = true; break; }
    pose = stepTrue(pose, r.applied, cfg.dt);
  }
  EXPECT_TRUE(reached) << "stall-near-end never latched GOAL_REACHED";
  EXPECT_LT(pose.x, wall_x + 0.05) << "robot drove past the wall";
  // Confirm it was the STALL path, not at_end: still short of the path endpoint.
  EXPECT_GT(std::hypot(1.2 - pose.x, 0.0 - pose.y), cfg.xy_goal_tolerance);
}

// ---------------------------------------------------------------------------
// 10b. Tight stall band: the SAME near-end stall that latched under a 0.5 m band
//      must NOT latch under the default reach_stall_radius (0.10 m), because the
//      footprint filter holds the robot >0.10 m short of the path endpoint.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, StallOutsideTightBandDoesNotLatch) {
  MpcConfig cfg;                         // reach_stall_radius defaults 0.10
  cfg.safety.enabled = true;
  cfg.reach_stall_cycles = 5;
  LocalPlanner lp(cfg);

  const double wall_x = 1.0;
  lp.setPath(straightPathX(1.2, 0.1));   // endpoint (1.2, 0) is INSIDE the wall
  lp.setCostmap(corridorWithWall(0.6, 0.0, 4.0, wall_x, cfg));
  Pose2D goal;
  goal.x = 1.2;
  lp.setGoal(goal);

  Pose2D pose;
  bool ever_reached = false;
  for (int i = 0; i < 200; ++i) {
    const MpcResult r = lp.compute(pose);
    if (r.status == ControllerState::GOAL_REACHED) ever_reached = true;
    if (r.status == ControllerState::TRACKING)
      pose = stepTrue(pose, r.applied, cfg.dt);
  }
  EXPECT_FALSE(ever_reached)
      << "latched inside a 0.10 m band while stalled >0.10 m short of the end";
  // Confirm the robot really did stall well outside the tight band.
  EXPECT_GT(std::hypot(1.2 - pose.x, 0.0 - pose.y), 0.10);
}

// ---------------------------------------------------------------------------
// 12. Reachable goal: arrival is judged against the RAW goal, not the snapped
//     path endpoint. Robot sitting on a path endpoint 0.07 m short of the raw
//     goal must NOT latch (xy err 0.07 > 0.05 tol); at the raw goal it latches.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, ReachableGoalJudgedAgainstRawGoal) {
  MpcConfig cfg;                         // reach_on_path_end true, reach_goal_gap 0.15
  LocalPlanner lp(cfg);
  lp.setCostmap(freeCostmap(1.0, 0.0, 3.0, cfg));
  lp.setPath(straightPathX(2.0, 0.1));   // path endpoint (2.0, 0)
  Pose2D goal;
  goal.x = 2.07;                         // raw goal 0.07 m beyond the endpoint (reachable: <=0.15)
  lp.setGoal(goal);

  // Sitting on the path endpoint is 0.07 m from the raw goal -> not within 0.05 tol.
  Pose2D at_end; at_end.x = 2.0;
  EXPECT_EQ(lp.compute(at_end).status, ControllerState::TRACKING);

  // Sitting on the raw goal -> reached.
  Pose2D at_goal; at_goal.x = 2.07;
  EXPECT_EQ(lp.compute(at_goal).status, ControllerState::GOAL_REACHED);
}

// ---------------------------------------------------------------------------
// 11. Stall AWAY from the end is a real block, not success. A wall far before a
//     distant path end stalls the robot OUTSIDE reach_stall_radius -> must NOT
//     latch GOAL_REACHED (would route to g1_nav recovery in the real stack).
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, StallAwayFromEndDoesNotLatch) {
  MpcConfig cfg;
  cfg.safety.enabled = true;
  cfg.reach_stall_cycles = 5;
  LocalPlanner lp(cfg);

  const double wall_x = 1.0;
  lp.setPath(straightPathX(3.0, 0.1));   // far endpoint (3.0, 0)
  lp.setCostmap(corridorWithWall(1.5, 0.0, 5.0, wall_x, cfg));
  Pose2D goal;
  goal.x = 3.0;
  lp.setGoal(goal);

  Pose2D pose;
  bool ever_reached = false;
  for (int i = 0; i < 200; ++i) {
    const MpcResult r = lp.compute(pose);
    if (r.status == ControllerState::GOAL_REACHED) ever_reached = true;
    if (r.status == ControllerState::TRACKING)
      pose = stepTrue(pose, r.applied, cfg.dt);
  }
  EXPECT_FALSE(ever_reached) << "falsely latched GOAL_REACHED on a mid-path block";
  EXPECT_LT(pose.x, wall_x + 0.2) << "robot drove through the wall";
}

// ---------------------------------------------------------------------------
// 12. Obstacle-nominal damping: with damping=1 the nominal collapses to the
//     reference, so the warm-started SECOND compute does NOT bow further than the
//     first (the limit-cycle amplifier is off). Opposite of WarmStartStatePersists.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, NominalDampingDisablesWarmStartAmplification) {
  MpcConfig cfg;
  cfg.obstacle_nominal_damping = 1.0;  // nominal == reference every cycle
  LocalPlanner lp(cfg);

  // Reuse the off-line-of-centre obstacle from WarmStartStatePersists.
  const double res = 0.1, half = 3.0;
  const int n = static_cast<int>(std::round(2.0 * half / res));
  const double ox = -half, oy = -half;
  std::vector<int8_t> data(static_cast<size_t>(n) * static_cast<size_t>(n), 0);
  auto setCell = [&](double wx, double wy, int8_t v) {
    int mx = static_cast<int>(std::floor((wx - ox) / res));
    int my = static_cast<int>(std::floor((wy - oy) / res));
    if (mx >= 0 && mx < n && my >= 0 && my < n) {
      int8_t& cell = data[static_cast<size_t>(my) * n + mx];
      if (v > cell) cell = v;
    }
  };
  for (double wx = -0.1; wx <= 1.1; wx += res * 0.5)
    for (double wy = -0.7; wy <= 0.8; wy += res * 0.5) {
      const double d = std::hypot(wx - 0.5, wy - 0.05);
      if (d <= 0.18) setCell(wx, wy, 100);
    }
  lp.setCostmap(CostmapView(res, ox, oy, n, n, data, cfg.lethal_cost,
                            cfg.treat_unknown_as_obstacle));
  lp.setPath(straightPathX(2.0, 0.1));
  Pose2D goal; goal.x = 2.0; lp.setGoal(goal);

  auto maxLat = [](const std::vector<Pose2D>& pred) {
    double m = 0.0; for (const Pose2D& p : pred) m = std::max(m, std::abs(p.y));
    return m;
  };
  const MpcResult r1 = lp.compute(Pose2D{});
  ASSERT_EQ(r1.status, ControllerState::TRACKING);
  const MpcResult r2 = lp.compute(Pose2D{});
  ASSERT_EQ(r2.status, ControllerState::TRACKING);
  // Under full damping the second cycle does not amplify the bow-out.
  EXPECT_LE(maxLat(r2.predicted), maxLat(r1.predicted) + 1e-3);
}

// ---------------------------------------------------------------------------
// 13. Near-obstacle strafe suppression: a planner near an inflated obstacle has
//     its applied vy scaled down vs the same setup with suppression off.
// ---------------------------------------------------------------------------
TEST(LocalPlannerTest, VySuppressionScalesLateralNearObstacle) {
  auto run = [](double scale) {
    MpcConfig cfg;
    cfg.vy_suppress_scale = scale;     // 1.0 = off, < 1.0 = suppress
    cfg.vy_suppress_cost  = 0.3;
    cfg.w_lateral = 0.0;               // let the obstacle gradient drive vy freely
    LocalPlanner lp(cfg);
    // Obstacle to the LEFT (+y) so the down-gradient push commands vy < 0. A lethal
    // block (100) at wy>=0.25 drives the gradient; an inflation band (80<100, NOT a
    // collision) at wy in [0.05,0.25) lets the footprint sample a high cost so the
    // C3 gate triggers without the faithful disk check overlapping a lethal cell.
    const double res = 0.1, half = 3.0;
    const int n = static_cast<int>(std::round(2.0 * half / res));
    const double ox = -half, oy = -half;
    std::vector<int8_t> data(static_cast<size_t>(n) * static_cast<size_t>(n), 0);
    auto setCell = [&](double wx, double wy, int8_t v) {
      int mx = static_cast<int>(std::floor((wx - ox) / res));
      int my = static_cast<int>(std::floor((wy - oy) / res));
      if (mx >= 0 && mx < n && my >= 0 && my < n) {
        int8_t& cell = data[static_cast<size_t>(my) * n + mx];
        if (v > cell) cell = v;
      }
    };
    for (double wx = -0.2; wx <= 0.6; wx += res * 0.5)
      for (double wy = 0.25; wy <= 0.6; wy += res * 0.5) setCell(wx, wy, 100);
    for (double wx = -0.2; wx <= 0.6; wx += res * 0.5)
      for (double wy = 0.05; wy < 0.25; wy += res * 0.5) setCell(wx, wy, 80);
    lp.setCostmap(CostmapView(res, ox, oy, n, n, data, cfg.lethal_cost,
                              cfg.treat_unknown_as_obstacle));
    lp.setPath(straightPathX(2.0, 0.1));
    Pose2D goal; goal.x = 2.0; lp.setGoal(goal);
    return lp.compute(Pose2D{}).applied.vy;
  };
  const double vy_off = run(1.0);
  const double vy_on  = run(0.3);
  ASSERT_GT(std::abs(vy_off), 1e-6);                 // sanity: some lateral command
  EXPECT_LT(std::abs(vy_on), std::abs(vy_off) + 1e-9);
  EXPECT_NEAR(std::abs(vy_on), 0.3 * std::abs(vy_off), 1e-6);
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
