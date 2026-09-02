#pragma once

// mpc.h -- ROS-free linear-MPC core for g1_local_planner.
// No ROS headers. Uses Eigen (dense + sparse) and osqp-eigen only.
//
// Assembles and solves one Quadratic Program per control cycle (N=20, dt=0.1).
// The decision vector is the sparse state+control form
//   z = [ s_1 .. s_N , u_0 .. u_{N-1} ]   (dim 6N)
// with s_k = [x, y, yaw] in the map frame and u_j = [vx, vy, w] the body-frame
// command applied over [j, j+1). s_0 (the current pose) is a CONSTANT, not a
// variable. The body-to-map rotation is held fixed at R(theta0) over the whole
// horizon (LTI about the current heading), so the dynamics are linear.
//
// Cost:
//   tracking    : w_pos * ||p_k - p_ref_k||^2 + w_yaw * (yaw_k - yaw_ref_k)^2
//                 (step N uses the terminal weights w_pos_terminal / w_yaw_terminal)
//   obstacle    : w_obstacle * g_k . p_k, with g_k = costmap.gradient(p_nom_k)
//                 (linearised about a nominal, pushes p_k DOWN-gradient)
//   effort      : w_effort  * ||u_j||^2
//   strafe      : w_lateral * vy_j^2
//   rate        : w_rate    * ||u_j - u_{j-1}||^2, u_{-1} = last_cmd
//
// Obstacle nominal (warm-started real-time-iteration SQP):
//   The obstacle term is LINEARISED, so the gradient g_k must be sampled at some
//   nominal point p_nom_k. By default it is sampled at the reference point
//   refs[k-1] (the stateless single-iteration scheme). When the caller passes an
//   obstacle_nominal trajectory of size == cfg.horizon (the PREVIOUS cycle's
//   predicted states s_1..s_N), g_k is instead sampled at obstacle_nominal[k-1],
//   re-linearising the obstacle term about last cycle's predicted path. This is
//   the standard real-time-iteration SQP warm start: once the predicted path bows
//   off-centre, the next cycle samples a NONZERO lateral gradient where the path
//   actually is, so the down-gradient push grows and avoidance becomes self-
//   reinforcing across cycles (the LocalPlanner facade feeds last_predicted back
//   in for exactly this reason).
//   HEAD-ON-SYMMETRY CAVEAT: for a perfectly symmetric head-on obstacle centred on
//   the reference line, the lateral gradient g_y ~ 0 on the FIRST cycle (the
//   reference sits at the cost ridge where left/right cancel), so the warm start
//   has nothing to amplify until some asymmetry appears. The GLOBAL planner is the
//   primary avoider that routes around such blocks; the local MPC handles tracking
//   margin, dynamic obstacles, and smoothing through the inflation halo, not
//   breaking a perfect head-on symmetry on its own.
//
// Constraints (l <= A z <= u):
//   dynamics equality : s_1 - B u_0 = s_0 ; s_{k+1} - s_k - B u_k = 0
//   box on u          : limits from MpcConfig
//   slew on u         : |u_j - u_{j-1}| <= acc_lim * dt (u_{-1} = last_cmd)
// There is NO hard obstacle constraint (soft penalty only) so the QP is always
// feasible. On any solver failure solve() returns solved=false with a zero command
// and a zero rollout, and NEVER throws.

#include <vector>

#include "g1_local_planner/core/costmap_view.h"
#include "g1_local_planner/core/plan_types.h"
#include "g1_local_planner/core/robot_footprint.h"

namespace g1_local_planner {

class Mpc {
 public:
  explicit Mpc(const MpcConfig& cfg);

  // Replace the live config (weights, limits, v_ref). horizon / dt are read on
  // every solve() so a changed horizon is honoured on the next call.
  void setConfig(const MpcConfig& cfg);

  const MpcConfig& config() const { return cfg_; }

  // Set the robot footprint used to average the obstacle gradient over the
  // body's rotated circle centers at each horizon step. When set, the obstacle
  // term averages gradient samples over worldSamples(); when not set (the
  // default), the single center-point behavior is preserved.
  void setFootprint(const RobotFootprint& fp) { footprint_ = fp; has_footprint_ = true; }

  // Solve one MPC cycle.
  //   current  -- s_0, the current map pose (a constant in the QP)
  //   theta0   -- the fixed LTI linearisation heading (usually current.yaw)
  //   refs     -- exactly cfg.horizon reference states (p_ref + yaw_ref) from
  //               path_reference; refs[k] is the target for state s_{k+1}
  //   last_cmd -- u_{-1}, the last applied command, seeds the rate cost
  //               and the k=0 slew constraint. CLAMPED componentwise into the
  //               velocity box at solve entry so a stale out-of-box last_cmd
  //               (e.g. after a live config change shrank the limits) can never
  //               make a k=0 slew row empty.
  //   costmap  -- the local-window CostmapView (obstacle gradient source)
  //   obstacle_nominal -- OPTIONAL warm-start nominal for the obstacle term. When
  //               non-null AND its size == cfg.horizon, the obstacle gradient for
  //               horizon step k is sampled at obstacle_nominal[k-1] (the previous
  //               cycle's predicted state) instead of at refs[k-1]. When null or
  //               the wrong size, falls back to sampling at the reference points
  //               (the original stateless behaviour). See the header comment for
  //               the real-time-iteration SQP rationale and head-on caveat.
  //   obstacle_weight_scale -- multiplies cfg.w_obstacle for THIS solve only
  //               (default 1.0). The LocalPlanner facade scales it toward 0 on the
  //               final approach to a reachable goal so the soft obstacle term does
  //               not limit-cycle the robot at the inflation boundary; tracking + the
  //               hard footprint safety filter then own the approach into a goal that
  //               sits in the costmap inscribed band.
  // Returns:
  //   .applied   = u_0 (zero on failure)
  //   .predicted = s_1 .. s_N (zero rollout on failure)
  //   .controls  = u_0 .. u_{N-1} (the full solved control sequence; empty on
  //                an early bail such as a bad reference size)
  //   .solved    = true on OSQP Solved / SolvedInaccurate, false otherwise
  //   .cost      = the QP objective value at the solution (0 on failure)
  MpcResult solve(const Pose2D& current, double theta0,
                  const std::vector<Pose2D>& refs,
                  const Twist2D& last_cmd,
                  const CostmapView& costmap,
                  const std::vector<Pose2D>* obstacle_nominal = nullptr,
                  double obstacle_weight_scale = 1.0);

 private:
  MpcConfig cfg_;
  RobotFootprint footprint_;
  bool has_footprint_ = false;
};

}  // namespace g1_local_planner
