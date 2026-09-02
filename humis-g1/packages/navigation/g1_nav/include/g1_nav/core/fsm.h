#pragma once
// fsm.h -- NavFsm: the g1_nav navigation state machine.
// ROS-free, std::thread-free. Pure transition function: (state, obs, now) -> NavDecision.
// See core-interface.md for authoritative transition semantics.

#include "g1_nav/core/nav_types.h"
#include "g1_nav/core/retry.h"

namespace g1_nav {

class NavFsm {
 public:
  explicit NavFsm(const NavConfig& cfg);

  // Live reconfigure; the next step() uses the new config.
  void setConfig(const NavConfig& cfg);

  // Current FSM state (before the next step).
  NavState state() const;

  // Force back to IDLE, clear goal and all timers.
  void reset();

  // Advance the FSM one tick. now = monotonic seconds (caller supplies; no clock calls
  // inside). Deterministic: (internal state, obs, now) -> decision.
  NavDecision step(const Observations& obs, double now);

 private:
  NavConfig cfg_;
  NavState  state_ = NavState::IDLE;

  // Active goal (set on new_goal, cleared on terminal/cancel).
  Pose2D goal_        = {};
  bool   have_goal_   = false;

  // Timestamps (all in the same monotonic domain as 'now').
  double goal_start_time_  = 0.0;  // set when a new goal is accepted
  double state_entry_time_ = 0.0;  // set when the FSM enters each non-IDLE state

  // Debounce: first time NO_PATH was seen in TRACK.
  // -1 = debounce inactive; set to `now` on first NO_PATH in TRACK.
  double no_path_since_ = -1.0;

  // RETRY timers (same monotonic domain as `now`).
  double retry_entry_time_   = 0.0;  // set on entering RETRY (budget clock)
  double last_attempt_time_ = 0.0;  // last re-forward-goal attempt in RETRY

  // Helper: true if the per-goal timeout is enabled and has elapsed.
  bool goalTimedOut(double now) const;

  // Enter RETRY: anchor the budget/attempt clocks and emit an immediate first
  // re-forward-goal attempt (hold, keep local enabled).
  NavDecision enterRetry(double now);

  // Helpers that build a terminal NavDecision (SUCCEEDED or ABORTED).
  NavDecision makeTerminal(NavState terminal_state, NavOutcome outcome);

  // Helper: build the IDLE NavDecision (clears all intents).
  static NavDecision makeIdle();
};

} // namespace g1_nav
