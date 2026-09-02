#pragma once
// nav_types.h -- shared types for the g1_nav core (FSM, watchdog).
// ROS-free. namespace g1_nav.

#include <cstdint>

namespace g1_nav {

struct Pose2D  { double x = 0.0, y = 0.0, yaw = 0.0; };
struct Twist2D { double vx = 0.0, vy = 0.0, w = 0.0; };   // body frame, matches /cmd_vel

// g1_nav FSM state. Published as std_msgs/Int8 on /nav/state; also the action
// feedback.state. Values are stable wire constants.
// RECOVER = 4 is RESERVED: the recovery mechanism was removed (soft failures route to
// RETRY); the value is retained for /nav/state wire stability and is never emitted.
enum class NavState : int8_t {
  IDLE = 0, PLAN = 1, TRACK = 2, REPLAN = 3, RECOVER = 4, SUCCEEDED = 5, ABORTED = 6,
  RETRY = 7   // blocked goal: hold + re-plan on a cadence within retry_budget
};

// Mirror of g1_local_planner ControllerState (the std_msgs/Int8 it publishes on
// /g1_local_planner/state). Keep these values identical to that enum.
enum class LocalState : int8_t {
  IDLE = 0, TRACKING = 1, GOAL_REACHED = 2, NO_PATH = 3, NO_GOAL = 4,
  NO_COSTMAP = 5, STUCK = 6
};

// How the node's single /cmd_vel publisher should source its command this tick.
enum class MuxMode : uint8_t { ZERO = 0, RELAY_TRACK = 1 };

// Terminal outcome; values map 1:1 to NavigateTo.action Result OUTCOME_* constants.
// NONE = not terminal this tick.
enum class NavOutcome : uint8_t {
  SUCCEEDED = 0, ABORTED_NO_PATH = 1, ABORTED_STUCK = 2, ABORTED_TIMEOUT = 3,
  ABORTED_RECOVERY_EXHAUSTED = 4,  // reserved: recovery removed, never produced
  PREEMPTED = 5, REJECTED = 6,
  ABORTED_BLOCKED_TIMEOUT = 7, NONE = 255
};

// ---- Config sub-structs (all fields have these code defaults) ----

// FSM-level timeouts + toggles.
struct NavConfig {
  double plan_timeout   = 5.0;    // s: PLAN without TRACKING -> RETRY
  double replan_timeout = 3.0;    // s: REPLAN without a fresh path -> RETRY
  double goal_timeout   = 9999.0; // s: per-goal wall time for the ACTIVE phases -> ABORTED_TIMEOUT (0 = unbounded; RETRY ignores it)
  double no_path_grace  = 1.0;    // s: TRACK must see NO_PATH this long before -> REPLAN (debounce)
  bool   auto_arm       = true;   // emit arm on accept + halt on terminal/cancel

  // Blocked-goal RETRY (the sole soft-failure handler). PLAN/REPLAN timeout and TRACK
  // STUCK / no_progress all funnel into RETRY: hold position, re-forward the goal every
  // retry_replan_interval, resume on a fresh path, and abort with
  // ABORTED_BLOCKED_TIMEOUT only after retry_budget elapses.
  double retry_budget          = 120.0;  // s: RETRY wall-clock budget (<= 0 = unbounded)
  double retry_replan_interval = 1.0;    // s: RETRY re-forward-goal cadence
};

// ProgressWatchdog params.
struct WatchdogConfig {
  double stuck_window = 8.0;   // s: rolling window; sized so the local stall-latch (~0.75 s) finishes before no_progress fires
  double stuck_dist   = 0.10;  // m: min net displacement in the window to count as progress
};

// ---- FSM I/O ----

// Everything NavFsm::step needs. The NODE fills this each tick.
struct Observations {
  // Edge events: the node sets these true for EXACTLY ONE step() when they occur.
  bool   new_goal = false;        // a goal was accepted (action OR RViz convenience)
  Pose2D goal     = {};           // the accepted goal pose (valid iff new_goal)
  bool   cancel   = false;        // explicit cancel/preempt request

  // Continuous signals.
  LocalState local_state    = LocalState::NO_GOAL;  // latest /g1_local_planner/state
  bool       costmap_available = false;             // /nav/local_costmap seen recently
  bool       have_pose      = false;
  Pose2D     pose           = {};                   // map->base_link (feedback + watchdog)

  // Booleans produced by the Task-3 helpers; the node ticks them and feeds results in.
  bool no_progress        = false;  // ProgressWatchdog fired (in TRACK)

  bool loco_halted        = false;  // /g1/loco_status halted latch (informational only)
};

// What the FSM tells the node to do this tick. Intents are edge-triggered (true only
// on the tick the FSM wants the action performed).
struct NavDecision {
  NavState next_state = NavState::IDLE;
  MuxMode  mux        = MuxMode::ZERO;

  bool forward_goal   = false;  // publish the active goal to /nav/goal
  bool enable_local   = false;  // local planner ~enable true
  bool disable_local  = false;  // local planner ~enable false
  bool reset_watchdog = false;  // reset the ProgressWatchdog (entered TRACK / new goal)
  bool arm            = false;  // /g1/arm   (only emitted when cfg.auto_arm)
  bool halt           = false;  // /g1/halt  (only emitted when cfg.auto_arm)

  bool       finish_action = false;          // set action result, then FSM returns to IDLE
  NavOutcome outcome       = NavOutcome::NONE;// valid iff finish_action
};

} // namespace g1_nav
