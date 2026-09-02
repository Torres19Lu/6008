// Unit tests for the MPC core: QP assembly + osqp-eigen solve.
//
// Tests are MEANINGFUL: open-field straight tracking, box-limit respect across
// the full control sequence, slew respect on the first step, obstacle-gradient
// avoidance, always-feasible under a dense lethal field, and yaw unwrap (rotate
// the short way across +-pi). OSQP is iterative so limit assertions are
// inequalities and equality checks use loose tolerances.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "g1_local_planner/core/costmap_view.h"
#include "g1_local_planner/core/plan_types.h"
#include "g1_local_planner/core/robot_footprint.h"
#include "g1_local_planner/planner/mpc.h"

using g1_local_planner::CostmapView;
using g1_local_planner::Mpc;
using g1_local_planner::MpcConfig;
using g1_local_planner::MpcResult;
using g1_local_planner::Pose2D;
using g1_local_planner::RobotFootprint;
using g1_local_planner::Twist2D;

namespace {

// A uniform-free costmap covering a generous area around the origin so world
// queries along the reference are in-bounds (out-of-bounds returns lethal_cost,
// which we do NOT want for the free-field tests).
//   half_m metres each side of the centre, resolution 0.1 m, all cells = 0.
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

// Build N reference poses spaced along +x from the origin at the given step
// (metres per horizon step). yaw_ref = 0 (face +x).
std::vector<Pose2D> straightRefX(int n, double step) {
  std::vector<Pose2D> refs;
  refs.reserve(static_cast<size_t>(n));
  for (int k = 1; k <= n; ++k) {
    Pose2D p;
    p.x = step * static_cast<double>(k);
    p.y = 0.0;
    p.yaw = 0.0;
    refs.push_back(p);
  }
  return refs;
}

// Highest costmap cost over a set of points (for the avoidance test).
double maxCost(const CostmapView& cm, const std::vector<Pose2D>& pts) {
  double m = 0.0;
  for (const Pose2D& p : pts) m = std::max(m, cm.cost(p.x, p.y));
  return m;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. Open field: a straight +x reference is tracked forward.
// ---------------------------------------------------------------------------
TEST(MpcTest, OpenFieldTracksStraight) {
  MpcConfig cfg;  // defaults
  Mpc mpc(cfg);

  Pose2D current;  // origin, yaw 0
  const double theta0 = 0.0;
  Twist2D last_cmd;  // zero

  // Reference advances at v_ref * dt per step (the natural cruise spacing).
  const auto refs = straightRefX(cfg.horizon, cfg.v_ref * cfg.dt);
  const CostmapView cm = freeCostmap(1.0, 0.0, 3.0, cfg);

  const MpcResult r = mpc.solve(current, theta0, refs, last_cmd, cm);

  ASSERT_TRUE(r.solved);
  EXPECT_GT(r.applied.vx, 0.05);                 // moving forward
  EXPECT_LT(std::abs(r.applied.vy), 0.05);       // little strafe
  EXPECT_LT(std::abs(r.applied.w), 0.05);        // little yaw
  ASSERT_EQ(r.predicted.size(),
            static_cast<size_t>(cfg.horizon));
  EXPECT_GT(r.predicted.back().x, 0.1);          // endpoint advanced along +x
  EXPECT_LT(std::abs(r.predicted.back().y), 0.1);
}

// ---------------------------------------------------------------------------
// 2. Box limits respected on EVERY control in the solution, even when the
//    reference demands far more speed than the limits allow.
// ---------------------------------------------------------------------------
TEST(MpcTest, BoxLimitsRespected) {
  MpcConfig cfg;
  Mpc mpc(cfg);

  Pose2D current;
  const double theta0 = 0.0;
  Twist2D last_cmd;

  // A reference demanding huge speed: 5 m per step (50 m/s) along +x.
  const auto refs = straightRefX(cfg.horizon, 5.0);
  const CostmapView cm = freeCostmap(50.0, 0.0, 60.0, cfg);

  const MpcResult r = mpc.solve(current, theta0, refs, last_cmd, cm);
  ASSERT_TRUE(r.solved);
  ASSERT_EQ(r.controls.size(), static_cast<size_t>(cfg.horizon));

  const double eps = 1e-3;  // OSQP feasibility slack
  for (const Twist2D& u : r.controls) {
    EXPECT_GE(u.vx, cfg.min_vx - eps);
    EXPECT_LE(u.vx, cfg.max_vx + eps);
    EXPECT_LE(std::abs(u.vy), cfg.max_vy + eps);
    EXPECT_LE(std::abs(u.w), cfg.max_wz + eps);
  }
  // The huge-demand case should drive vx up to (near) the cap.
  EXPECT_GT(r.controls.back().vx, cfg.max_vx - 0.05);
}

// ---------------------------------------------------------------------------
// 3. Slew respected on the first step (relative to last_cmd).
// ---------------------------------------------------------------------------
TEST(MpcTest, SlewRespected) {
  MpcConfig cfg;
  Mpc mpc(cfg);

  const double theta0 = 0.0;
  const double slew_x = cfg.acc_lim_x * cfg.dt;  // 1.5 * 0.1 = 0.15
  const double eps = 1e-3;

  // (a) from rest, a far ref: the first vx step cannot exceed acc_lim_x*dt.
  {
    Pose2D current;
    Twist2D last_cmd;  // zero
    const auto refs = straightRefX(cfg.horizon, 5.0);
    const CostmapView cm = freeCostmap(50.0, 0.0, 60.0, cfg);
    const MpcResult r = mpc.solve(current, theta0, refs, last_cmd, cm);
    ASSERT_TRUE(r.solved);
    EXPECT_LE(r.applied.vx, slew_x + eps);
    EXPECT_GE(r.applied.vx, -eps);
  }

  // (b) starting at vx=0.5, a decel-demanding ref (reference behind, at -x):
  //     |applied.vx - 0.5| <= acc_lim_x*dt.
  {
    Pose2D current;
    Twist2D last_cmd;
    last_cmd.vx = 0.5;
    // Reference pulls strongly backward so the optimiser wants to brake hard.
    std::vector<Pose2D> refs;
    for (int k = 1; k <= cfg.horizon; ++k) {
      Pose2D p;
      p.x = -5.0;  // far behind
      p.y = 0.0;
      p.yaw = 0.0;
      refs.push_back(p);
    }
    const CostmapView cm = freeCostmap(-2.0, 0.0, 60.0, cfg);
    const MpcResult r = mpc.solve(current, theta0, refs, last_cmd, cm);
    ASSERT_TRUE(r.solved);
    EXPECT_LE(std::abs(r.applied.vx - 0.5), slew_x + eps);
  }
}

// ---------------------------------------------------------------------------
// 4. Obstacle push: a lethal block straddling the straight reference line; the
//    predicted trajectory must steer AWAY (lower max cost than the raw refs).
// ---------------------------------------------------------------------------
TEST(MpcTest, ObstaclePush) {
  MpcConfig cfg;
  // Make avoidance dominant and tracking soft enough that the path can bow
  // around the block. Defaults already favour obstacle (50) over pos (10); we
  // raise w_obstacle and soften w_pos so the single-iteration linearised push
  // (sampled at the on-line reference points) drives the trajectory clearly
  // off the lethal core into the free corridor.
  cfg.w_obstacle = 400.0;
  cfg.w_pos = 5.0;
  Mpc mpc(cfg);

  Pose2D current;
  const double theta0 = 0.0;
  Twist2D last_cmd;

  // Straight reference along +x; the robot would pass through ~(0.5, 0).
  const auto refs = straightRefX(cfg.horizon, cfg.v_ref * cfg.dt);

  // Costmap: free everywhere except a lethal square around (0.5, 0) blocking
  // the reference line, with a free detour available above/below in y.
  const double res = 0.1;
  const double half = 3.0;
  const int n = static_cast<int>(std::round(2.0 * half / res));
  const double ox = current.x - half;
  const double oy = current.y - half;
  std::vector<int8_t> data(static_cast<size_t>(n) * static_cast<size_t>(n), 0);

  // Lethal block: a lethal core within radius 0.18 m of (0.5, 0), surrounded by
  // a smooth inflation halo that DECAYS with distance from value ~99 at the core
  // edge down to value 1 at radius 0.55 m, so the costmap gradient is smooth and
  // reaches the nominal reference points (review Minor 2: there is no flat
  // "value 60" halo; the halo is the linear-in-distance decay set just below).
  auto setCell = [&](double wx, double wy, int8_t v) {
    int mx = static_cast<int>(std::floor((wx - ox) / res));
    int my = static_cast<int>(std::floor((wy - oy) / res));
    if (mx >= 0 && mx < n && my >= 0 && my < n) {
      int8_t& cell = data[static_cast<size_t>(my) * n + mx];
      if (v > cell) cell = v;
    }
  };
  for (double wx = 0.0; wx <= 1.0; wx += res * 0.5) {
    for (double wy = -0.6; wy <= 0.6; wy += res * 0.5) {
      const double dx = wx - 0.5;
      const double dy = wy - 0.0;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d <= 0.18) {
        setCell(wx, wy, 100);              // lethal core
      } else if (d <= 0.55) {
        // smooth halo decaying with distance (99 .. 1)
        const int8_t v = static_cast<int8_t>(
            std::max(1.0, std::round(99.0 * (1.0 - (d - 0.18) / (0.55 - 0.18)))));
        setCell(wx, wy, v);
      }
    }
  }
  const CostmapView cm(res, ox, oy, n, n, data, cfg.lethal_cost,
                       cfg.treat_unknown_as_obstacle);

  const MpcResult r = mpc.solve(current, theta0, refs, last_cmd, cm);
  ASSERT_TRUE(r.solved);

  const double cost_refs = maxCost(cm, refs);
  const double cost_pred = maxCost(cm, r.predicted);

  // The reference line passes through the lethal core, so its max cost is high.
  EXPECT_GT(cost_refs, 0.9);
  // The predicted trajectory must be pushed off it: strictly lower peak cost.
  EXPECT_LT(cost_pred, cost_refs);
  // And it must genuinely clear the lethal core (no predicted point lands on a
  // lethal cell) given the free detour above/below the block.
  EXPECT_LT(cost_pred, 0.95);

  // The avoidance is real motion, not just a cost drop: the trajectory bows
  // laterally off the straight reference line by a meaningful margin.
  double max_lat = 0.0;
  for (const Pose2D& p : r.predicted) max_lat = std::max(max_lat, std::abs(p.y));
  EXPECT_GT(max_lat, 0.1);
}

// ---------------------------------------------------------------------------
// Terminal obstacle relax: obstacle_weight_scale == 0 fully disables the soft
// obstacle push, so the SAME block that bows ObstaclePush is tracked STRAIGHT
// through. This is the terminal-approach behaviour: near a reachable goal the
// soft term is suppressed and tracking (+ the hard footprint safety filter, not
// modelled here) owns the final approach into a goal sitting in the inscribed band.
// ---------------------------------------------------------------------------

TEST(MpcTest, ObstacleWeightScaleZeroDisablesPush) {
  MpcConfig cfg;
  cfg.w_obstacle = 400.0;
  cfg.w_pos = 5.0;
  Mpc mpc(cfg);

  Pose2D current;
  const double theta0 = 0.0;
  Twist2D last_cmd;
  const auto refs = straightRefX(cfg.horizon, cfg.v_ref * cfg.dt);

  // Same lethal block + inflation halo as ObstaclePush.
  const double res = 0.1;
  const double half = 3.0;
  const int n = static_cast<int>(std::round(2.0 * half / res));
  const double ox = current.x - half;
  const double oy = current.y - half;
  std::vector<int8_t> data(static_cast<size_t>(n) * static_cast<size_t>(n), 0);
  auto setCell = [&](double wx, double wy, int8_t v) {
    int mx = static_cast<int>(std::floor((wx - ox) / res));
    int my = static_cast<int>(std::floor((wy - oy) / res));
    if (mx >= 0 && mx < n && my >= 0 && my < n) {
      int8_t& cell = data[static_cast<size_t>(my) * n + mx];
      if (v > cell) cell = v;
    }
  };
  for (double wx = 0.0; wx <= 1.0; wx += res * 0.5) {
    for (double wy = -0.6; wy <= 0.6; wy += res * 0.5) {
      const double dx = wx - 0.5;
      const double dy = wy - 0.0;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d <= 0.18) {
        setCell(wx, wy, 100);
      } else if (d <= 0.55) {
        const int8_t v = static_cast<int8_t>(
            std::max(1.0, std::round(99.0 * (1.0 - (d - 0.18) / (0.55 - 0.18)))));
        setCell(wx, wy, v);
      }
    }
  }
  const CostmapView cm(res, ox, oy, n, n, data, cfg.lethal_cost,
                       cfg.treat_unknown_as_obstacle);

  // scale = 1 bows off (baseline); scale = 0 tracks straight (no soft push).
  const MpcResult r_on =
      mpc.solve(current, theta0, refs, last_cmd, cm, nullptr, 1.0);
  const MpcResult r_off =
      mpc.solve(current, theta0, refs, last_cmd, cm, nullptr, 0.0);
  ASSERT_TRUE(r_on.solved);
  ASSERT_TRUE(r_off.solved);

  double lat_on = 0.0, lat_off = 0.0;
  for (const Pose2D& p : r_on.predicted) lat_on = std::max(lat_on, std::abs(p.y));
  for (const Pose2D& p : r_off.predicted) lat_off = std::max(lat_off, std::abs(p.y));

  EXPECT_GT(lat_on, 0.1);    // obstacle term ON: trajectory bows away
  EXPECT_LT(lat_off, 0.02);  // obstacle term scaled to 0: tracks straight through
}

// ---------------------------------------------------------------------------
// 5. Always feasible: a costmap lethal everywhere near the reference still
//    returns solved==true and never throws (soft penalty only).
// ---------------------------------------------------------------------------
TEST(MpcTest, AlwaysFeasible) {
  MpcConfig cfg;
  cfg.w_obstacle = 500.0;  // crank the obstacle push hard
  Mpc mpc(cfg);

  Pose2D current;
  const double theta0 = 0.0;
  Twist2D last_cmd;
  const auto refs = straightRefX(cfg.horizon, cfg.v_ref * cfg.dt);

  // Everything lethal (value 100) in a window around the reference.
  const double res = 0.1;
  const double half = 3.0;
  const int n = static_cast<int>(std::round(2.0 * half / res));
  std::vector<int8_t> data(static_cast<size_t>(n) * static_cast<size_t>(n), 100);
  const CostmapView cm(res, current.x - half, current.y - half, n, n, data,
                       cfg.lethal_cost, cfg.treat_unknown_as_obstacle);

  MpcResult r;
  ASSERT_NO_THROW(r = mpc.solve(current, theta0, refs, last_cmd, cm));
  EXPECT_TRUE(r.solved);
  // Box limits still hold under the strong push.
  ASSERT_EQ(r.controls.size(), static_cast<size_t>(cfg.horizon));
  const double eps = 1e-3;
  for (const Twist2D& u : r.controls) {
    EXPECT_LE(std::abs(u.vy), cfg.max_vy + eps);
    EXPECT_LE(std::abs(u.w), cfg.max_wz + eps);
    EXPECT_GE(u.vx, cfg.min_vx - eps);
    EXPECT_LE(u.vx, cfg.max_vx + eps);
  }
}

// ---------------------------------------------------------------------------
// 6. Yaw unwrap: the QP rotates the SHORT way across +-pi.
// ---------------------------------------------------------------------------
TEST(MpcTest, YawUnwrap) {
  MpcConfig cfg;
  Mpc mpc(cfg);

  // (a) theta0 = 0, reference yaw = +3.0 rad: rotate positively (short way is
  //     +3.0, not the -3.28 long way).
  {
    Pose2D current;  // yaw 0
    const double theta0 = 0.0;
    Twist2D last_cmd;
    std::vector<Pose2D> refs;
    for (int k = 1; k <= cfg.horizon; ++k) {
      Pose2D p;
      p.x = 0.0;
      p.y = 0.0;
      p.yaw = 3.0;  // within pi of theta0 -> unwrap leaves it at +3.0
      refs.push_back(p);
    }
    const CostmapView cm = freeCostmap(0.0, 0.0, 3.0, cfg);
    const MpcResult r = mpc.solve(current, theta0, refs, last_cmd, cm);
    ASSERT_TRUE(r.solved);
    EXPECT_GT(r.applied.w, 0.0);  // positive rotation, the short way
  }

  // (b) theta0 = 3.0, reference yaw = -3.0. Raw difference is -6.0 (long way,
  //     would spin negative ~2pi); the true short way is +0.2832 rad
  //     (-3.0 == +3.2832 unwrapped). With unwrap the QP rotates POSITIVE and
  //     small, NOT a near-2pi the-long-way negative command.
  {
    Pose2D current;
    current.yaw = 3.0;
    const double theta0 = 3.0;
    Twist2D last_cmd;
    std::vector<Pose2D> refs;
    for (int k = 1; k <= cfg.horizon; ++k) {
      Pose2D p;
      p.x = 0.0;
      p.y = 0.0;
      p.yaw = -3.0;
      refs.push_back(p);
    }
    const CostmapView cm = freeCostmap(0.0, 0.0, 3.0, cfg);
    const MpcResult r = mpc.solve(current, theta0, refs, last_cmd, cm);
    ASSERT_TRUE(r.solved);
    EXPECT_GT(r.applied.w, 0.0);   // rotates the short (positive) way
    EXPECT_LT(r.applied.w, 0.3);   // small, not a near-max long-way command
  }
}

// ---------------------------------------------------------------------------
// 7. Warm-started obstacle nominal sharpens avoidance (Part A).
//    Re-linearising the obstacle term about the PREVIOUS cycle's predicted path
//    (real-time-iteration SQP) makes the trajectory bow FURTHER off the obstacle
//    than the stateless single-iteration solve. We run an initial solve with no
//    nominal (predicted1, sampled at the reference) and a second solve passing
//    predicted1 as the obstacle nominal, and assert the second solve bows
//    STRICTLY further off (greater lateral deviation) AND has a STRICTLY lower
//    peak costmap cost: both halves of the task's "or" hold cleanly on the single
//    warm-started step.
//
//    The obstacle is offset slightly off the reference line (in +y) so the first
//    cycle already sees a small NONZERO lateral gradient to amplify. A perfectly
//    symmetric head-on block would give g_y ~ 0 on the first cycle (the head-on
//    caveat documented in mpc.h); the slight offset stands in for the asymmetry
//    that the global planner / real motion provides.
//
//    This test exercises the warm-start MECHANISM, so it sets an explicit MODERATE
//    obstacle weight (w_obstacle=50, w_pos=10) rather than relying on the production
//    DEFAULT weights. The production defaults are deliberately TRACKING-dominant
//    (w_obstacle=2 vs w_pos=10; the global path is the primary avoider) so a
//    single warm-started step does not bow far enough in this STATIC-obstacle
//    micro-scenario to strictly lower the peak cost, and a very strong weight
//    (the ObstaclePush regime, 400) over-bows on the first stateless solve so the
//    re-linearisation then RELAXES instead of sharpening. The moderate weight here
//    sits in the band where one warm-started step is a clean, monotone improvement,
//    which is exactly the re-linearisation property being asserted.
//
//    NOTE on convergence: with this moderate obstacle weight the warm start is strong
//    but UNDAMPED, so iterating it to a fixed point on a STATIC obstacle limit-cycles
//    (the path bows fully clear, the nominal lands in free space where g ~ 0, and
//    the next solve relaxes back to the reference, repeating). It is therefore the
//    single warm-started STEP that is the robust, monotone improvement, not an
//    N-step fixed point. In the closed loop the geometry shifts every cycle as the
//    robot moves, so the static limit cycle does not arise (ClosedLoopReachesGoal
//    in test_local_planner.cpp drives it end to end); this is why the facade feeds
//    last_predicted forward one cycle at a time rather than iterating to convergence.
// ---------------------------------------------------------------------------
TEST(MpcTest, WarmStartSharpensAvoidance) {
  MpcConfig cfg;
  // Moderate obstacle weight (the band where one warm-started step monotonically
  // sharpens avoidance on this static single-obstacle micro-scenario). Decoupled
  // from the production defaults, which are tracking-dominant and validated
  // end-to-end by ClosedLoopReachesGoal + the offline closed-loop gate.
  cfg.w_obstacle = 50.0;
  cfg.w_pos = 10.0;
  Mpc mpc(cfg);

  Pose2D current;  // origin, yaw 0
  const double theta0 = 0.0;
  Twist2D last_cmd;  // zero

  // Straight +x reference at the natural cruise spacing; the robot would pass
  // near (0.5, 0) at mid-horizon.
  const auto refs = straightRefX(cfg.horizon, cfg.v_ref * cfg.dt);

  // Costmap: a lethal block centred slightly OFF the line at (0.5, +0.05) with a
  // smooth decaying halo, so the reference points see a small but nonzero lateral
  // gradient (pushing -y). A free corridor exists below (in -y).
  const double res = 0.1;
  const double half = 3.0;
  const int n = static_cast<int>(std::round(2.0 * half / res));
  const double ox = current.x - half;
  const double oy = current.y - half;
  std::vector<int8_t> data(static_cast<size_t>(n) * static_cast<size_t>(n), 0);

  const double obs_cx = 0.5;
  const double obs_cy = 0.05;  // slight +y offset breaks the head-on symmetry
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
      const double dx = wx - obs_cx;
      const double dy = wy - obs_cy;
      const double d = std::sqrt(dx * dx + dy * dy);
      if (d <= 0.18) {
        setCell(wx, wy, 100);  // lethal core
      } else if (d <= 0.55) {
        const int8_t v = static_cast<int8_t>(
            std::max(1.0, std::round(99.0 * (1.0 - (d - 0.18) / (0.55 - 0.18)))));
        setCell(wx, wy, v);
      }
    }
  }
  const CostmapView cm(res, ox, oy, n, n, data, cfg.lethal_cost,
                       cfg.treat_unknown_as_obstacle);

  // Max lateral deviation (|y|) of a predicted trajectory.
  auto maxLat = [](const std::vector<Pose2D>& pred) {
    double m = 0.0;
    for (const Pose2D& p : pred) m = std::max(m, std::abs(p.y));
    return m;
  };

  // First solve: no warm-start nominal (gradient sampled at the reference line).
  const MpcResult r1 = mpc.solve(current, theta0, refs, last_cmd, cm, nullptr);
  ASSERT_TRUE(r1.solved);
  ASSERT_EQ(r1.predicted.size(), static_cast<size_t>(cfg.horizon));

  // Second solve: re-linearise the obstacle term about predicted1 (the warm-start
  // nominal). The path bowed off-centre in solve 1, so solve 2 samples a stronger
  // lateral gradient where the path actually is and pushes further off.
  const MpcResult r2 =
      mpc.solve(current, theta0, refs, last_cmd, cm, &r1.predicted);
  ASSERT_TRUE(r2.solved);
  ASSERT_EQ(r2.predicted.size(), static_cast<size_t>(cfg.horizon));

  // Bows STRICTLY further off the obstacle than the stateless solve...
  EXPECT_GT(maxLat(r2.predicted), maxLat(r1.predicted) + 1e-3)
      << "warm-started solve did not bow further than the stateless solve";
  // ...AND clears it better (strictly lower peak costmap cost).
  EXPECT_LT(maxCost(cm, r2.predicted), maxCost(cm, r1.predicted) - 1e-3)
      << "warm-started solve did not lower the peak costmap cost";
}

// ---------------------------------------------------------------------------
// 8. Footprint-averaged obstacle gradient pushes the WHOLE body off a wall.
//    The MPC is configured with an amplified obstacle weight so the bow is
//    observable. A lethal band on the +y side creates a gradient pointing -y;
//    the predicted path must bow toward -y (away from the wall).
// ---------------------------------------------------------------------------
TEST(Mpc, FootprintObstacleGradientPushesWholeBodyOffWall) {
  // FootprintConfig defaults: circle_count=5, lateral_width=0.5 m,
  // circle_radius=0.15 m. half_span = 0.5/2 - 0.15 = 0.10 m, so the 5 circles
  // are at body-frame y = {-0.10, -0.05, 0, +0.05, +0.10} m.
  //
  // The costmap places a lethal band starting at world y = 0.125 m
  // (cell row >= 33 with res=0.05, origin_y=-1.5). The single center at y=0 is
  // 0.125 m from the lethal edge -- too far for the gradient step (0.05 m) to
  // detect it (cost(+0.05)=0, cost(-0.05)=0, gy=0). Only the outermost
  // footprint circle at y=+0.10 m sits within one gradient step of the wall
  // (cost(+0.15m)=1.0, cost(+0.05m)=0, gy=10 cost/m). The footprint-averaged
  // gy = 10/5 = 2.0 cost/m, giving a net push q(iy) += 50*2=100 toward -y.
  MpcConfig cfg;
  cfg.w_obstacle = 50.0;  // amplify so the bow is observable on a single solve
  RobotFootprint fp;
  std::string err;
  RobotFootprint::build(cfg.footprint, &fp, &err);
  Mpc mpc(cfg);
  mpc.setFootprint(fp);

  // 60x60 grid, res=0.05 m, origin=(-1.5, -1.5).
  // y_world = -1.5 + cy * 0.05.  cy=33 -> y_world = -1.5 + 1.65 = 0.15 m.
  // Lethal band: cy >= 33  =>  y_world >= 0.15 m (just above the +0.10 m circle).
  const int W = 60;
  const int H = 60;
  std::vector<int8_t> data(static_cast<size_t>(W) * static_cast<size_t>(H), 0);
  for (int cy = 33; cy < H; ++cy)
    for (int cx = 0; cx < W; ++cx)
      data[static_cast<size_t>(cy) * W + cx] = 100;
  CostmapView cm(0.05, -1.5, -1.5, W, H, data, 1.0, true);

  // Straight +x reference at y=0, so the single-center gradient is zero at every
  // step. Only the footprint circle at +0.10 m sees the lethal edge.
  std::vector<Pose2D> refs(static_cast<size_t>(cfg.horizon));
  for (int k = 0; k < cfg.horizon; ++k) {
    refs[static_cast<size_t>(k)].x   = 0.05 * (k + 1);
    refs[static_cast<size_t>(k)].y   = 0.0;
    refs[static_cast<size_t>(k)].yaw = 0.0;
  }

  Pose2D cur;
  MpcResult r = mpc.solve(cur, 0.0, refs, Twist2D{}, cm, nullptr);
  ASSERT_TRUE(r.solved);
  // The footprint-averaged gradient pushes the path toward -y (away from the wall).
  EXPECT_LT(r.predicted.back().y, 0.0);
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
