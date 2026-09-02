#pragma once

// plan_types.h -- ROS-free shared types for g1_local_planner.
// No ROS headers. No Eigen. Pure structs/enums with code defaults.
//
// Cost weights in MpcConfig are PROVISIONAL defaults (tune on hardware).
// They will be refined during QP assembly and the tuning pass.
// Every field is a live dynamic_reconfigure knob except the structural
// params (horizon, dt) which are static YAML.

#include <cmath>
#include <cstdint>
#include <vector>

#include "g1_local_planner/core/robot_footprint.h"

namespace g1_local_planner {

// Wrap an angle into (-pi, pi]. Shared tiny util (used by the MPC yaw unwrap and
// the LocalPlanner facade goal-yaw error) so neither has to keep a file-local
// copy. Kept here in the ROS-free core header so both can include it cheaply.
inline double wrapToPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

// 2D pose in the map frame: position (x, y) in metres + heading in radians.
struct Pose2D {
  double x   = 0.0;
  double y   = 0.0;
  double yaw = 0.0;
};

// Body-frame velocity command, matching /cmd_vel layout:
//   vx  = forward  (linear.x)
//   vy  = lateral  (linear.y)
//   w   = yaw rate (angular.z)
struct Twist2D {
  double vx = 0.0;  // forward speed   (m/s)
  double vy = 0.0;  // lateral speed   (m/s, positive = robot-left)
  double w  = 0.0;  // yaw rate        (rad/s, positive = CCW)
};

// High-level controller state published as std_msgs/Int8 on ~state.
enum class ControllerState : int8_t {
  IDLE         = 0,  // no goal received yet; publishing zero cmd_vel
  TRACKING     = 1,  // actively following the global path
  GOAL_REACHED = 2,  // within xy_goal_tolerance + yaw_goal_tolerance
  NO_PATH      = 3,  // global path is empty or not yet received
  NO_GOAL      = 4,  // no /move_base_simple/goal received
  NO_COSTMAP   = 5,  // /nav/local_costmap not yet received
  STUCK        = 6,  // the MPC solve failed this cycle; zero cmd issued
};

// All MPC knobs with code defaults.
// The node YAML overrides these; dynamic_reconfigure overrides the tunable subset.
// NOTE: weights are provisional; refined in tuning.
struct MpcConfig {
  // --- Prediction horizon (structural; static YAML, not live) ---
  int    horizon = 20;    // number of prediction steps N
  double dt      = 0.1;   // step duration in seconds; MPC dt != control period (20 Hz)

  // --- Reference speed ---
  double v_ref = 0.5;     // nominal cruise speed (m/s) for arc-length reference

  // --- Velocity box constraints ---
  double max_vx = 0.5;    // maximum forward speed  (m/s)
  double min_vx = -0.3;   // minimum forward speed; allows slow reverse (m/s)
  double max_vy = 0.3;    // maximum lateral speed (m/s, symmetric)
  double max_wz = 0.8;    // maximum yaw rate (rad/s, symmetric)

  // --- Slew / acceleration limits, applied as |u_k - u_{k-1}| <= acc * dt ---
  double acc_lim_x     = 1.5;  // max longitudinal accel/decel (m/s^2)
  double acc_lim_y     = 1.0;  // max lateral accel/decel      (m/s^2)
  double acc_lim_theta = 1.5;  // max yaw angular accel        (rad/s^2)

  // --- Goal tolerances ---
  double xy_goal_tolerance  = 0.05;  // position tolerance for GOAL_REACHED (m)
  double yaw_goal_tolerance = 0.03;  // heading tolerance for GOAL_REACHED (rad)

  // --- Terminal behaviour ---
  double goal_align_radius = 0.5;  // switch heading ref to goal yaw + decelerate (m)

  // --- Clean termination near obstacles (Track A) ---
  // GOAL_REACHED is judged against the global path ENDPOINT (the planner's best
  // reachable pose), not the raw goal which may sit inside an obstacle's inscribed
  // band. A footprint-limited stall inside reach_stall_radius also latches reached.
  // Stall = NO PROGRESS toward the path end (robust to the limit-cycle oscillation
  // a hard block induces; a speed threshold is not, the robot bounces in place).
  // reach_stall_radius is the near-end gate, decoupled from goal_align_radius (which
  // governs only the approach deceleration + heading switch).
  bool   reach_on_path_end    = true;   // false restores the raw-goal reached check
  double reach_stall_progress = 0.01;   // m; min reduction in dist-to-path-end to count as progress
  int    reach_stall_cycles   = 15;     // consecutive no-progress near-end cycles to latch (15 @ 20 Hz = 0.75 s)
  double reach_stall_radius   = 0.15;   // m; near-end gate for the stall latch (~3x xy_goal_tolerance; reaches the footprint-limited stall pose near a table)
  double reach_goal_gap       = 0.15;   // m; if path endpoint is within this of the raw goal, judge + track the RAW goal (else fall back to the path endpoint)

  // --- Terminal obstacle relax (Track A; final approach into an inscribed-band goal) ---
  // A teleop-safe goal can sit in the costmap INSCRIBED halo (cost 99, ~robot_radius
  // from a real obstacle). The MPC soft obstacle term then repels the wide footprint at
  // the inflation boundary and the robot limit-cycles ~0.5 m out without converging.
  // Near a REACHABLE goal (path end within reach_goal_gap of the raw goal) and within
  // goal_obstacle_relax_radius of it, scale w_obstacle by goal_obstacle_relax_scale so
  // tracking drives the center in; the hard footprint safety filter stays the collision
  // guarantee (the global path already vetted the corridor). radius 0 disables.
  double goal_obstacle_relax_radius = 0.6;  // m; 0 disables the terminal relax
  double goal_obstacle_relax_scale  = 0.0;  // [0,1]; w_obstacle multiplier inside the radius

  // --- Speed regulation (live; shapes the arc-length reference speed) ---
  double a_lat_max          = 0.5;   // max lateral accel for the curvature regulator (m/s^2)
  double a_decel            = 0.2;   // approach decel-to-stop rate (m/s^2); lower = gentler/earlier slow-down into the goal (<= executable decel)
  double curv_lookahead     = 0.6;   // forward arc length scanned for path curvature (m)
  double heading_slow_start = 0.3;   // heading error where translation slowdown begins (rad)
  double heading_slow_full  = 1.0;   // heading error at which the speed factor hits the floor (rad)
  double heading_slow_floor = 0.0;   // lower bound on the heading speed factor in [0,1]
  double v_min_move         = 0.05;  // floor for the cruise regulators (NOT the approach ramp) (m/s)

  // --- Cost weights (tuned in the offline-gate tuning pass) ---
  // TRACKING-DOMINANT, mirroring the reference DWA (g1_navigation:
  // path_distance_bias=32, goal_distance_bias=24, occdist_scale=0.02 -- the obstacle
  // term is a SECONDARY safety nudge ~1000x smaller than tracking, because the global
  // path is the PRIMARY avoider). The obstacle term is the LINEAR penalty
  // w_obstacle * grad(cost) . p_k, so the predicted path settles ~ (w_obstacle/(2*w_pos))
  // * grad off the reference. The inflation-aware global path LEGITIMATELY runs through
  // the inflation band (1..99) on tight maps, where grad is large and consistent; an
  // obstacle-dominant weight (the old w_obstacle=50 vs w_pos=10) shoved the whole
  // predicted trajectory meters off that path, the warm start reinforced it, and the
  // robot STALLED (offline gate: only 8/28 inflation-grazing pairs reached the goal).
  // Keeping w_obstacle small lets the MPC HUG the inflation band as the global planner
  // intended, deviating only a few cm to keep tracking error off genuinely lethal cells
  // and to dodge dynamic obstacles the global path does not know about.
  double w_pos      = 10.0;  // path-tracking position weight (penalises p_k - p_ref_k)
  double w_yaw      =  2.0;  // heading weight (face travel direction / goal yaw)
  double w_obstacle =  2.0;  // linearised obstacle penalty (push down-gradient); SOFT,
                             // secondary to tracking (the global path is the avoider)
  double w_effort   =  0.1;  // control effort weight (penalises ||u_k||^2)
  double w_lateral  =  2.0;  // strafe penalty (penalises vy_k^2)
  double w_rate     =  0.5;  // control rate weight (penalises ||u_k - u_{k-1}||^2)

  // --- Terminal weights (higher emphasis on final state) ---
  double w_pos_terminal = 30.0;  // terminal position weight (step N)
  double w_yaw_terminal =  8.0;  // terminal heading weight  (step N)

  // --- Oscillation control (Track C) ---
  // Blend the warm-start obstacle nominal toward the reference each cycle to break
  // the bow-clear -> gradient-zero -> snap-back limit cycle. 0 = pure warm start
  // (legacy), 1 = nominal == reference (no warm-start memory).
  double obstacle_nominal_damping = 0.5;
  // Near-obstacle strafe suppression: when the max footprint-sample normalized cost
  // is >= vy_suppress_cost, scale the applied lateral velocity by vy_suppress_scale.
  double vy_suppress_cost  = 0.5;  // normalized cost [0,1] gate (1.0 = effectively off)
  double vy_suppress_scale = 0.6;  // factor applied to vy near obstacles (1.0 = off)

  // --- Obstacle cost normalization (used by CostmapView) ---
  double lethal_cost = 1.0;               // normalized cost assigned to OccupancyGrid value 100
                                          // (lethal/inscribed) or out-of-window cells
  bool treat_unknown_as_obstacle = true;  // if true, value -1 (unknown) -> lethal_cost; else 0.0

  // --- Footprint + safety filter ---
  FootprintConfig footprint;   // body-frame circle array (see robot_footprint.h)
  SafetyConfig    safety;      // post-MPC footprint safety filter (see footprint_safety.h)
  double          control_dt = 0.05;  // control period (s); 1/control_rate (default 20 Hz)
};

// Return value of one MPC solve cycle.
struct MpcResult {
  ControllerState   status    = ControllerState::IDLE;  // controller state after this cycle
  Twist2D           applied;                             // the u_0 command applied to /cmd_vel
  std::vector<Pose2D> predicted;                         // predicted states s_1 .. s_N (map frame)
  std::vector<Twist2D> controls;                         // full solved control sequence u_0 .. u_{N-1}
  double            cost      = 0.0;                    // total QP objective value at solution
  bool              solved    = false;                   // true if OSQP returned OPTIMAL/near-optimal
};

}  // namespace g1_local_planner
