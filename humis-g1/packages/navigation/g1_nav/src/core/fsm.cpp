// fsm.cpp -- NavFsm implementation. See core-interface.md for authoritative semantics.
// ROS-free, std::thread-free.

#include "g1_nav/core/fsm.h"

namespace g1_nav {

// ---------------------------------------------------------------------------
// Construction / config
// ---------------------------------------------------------------------------

NavFsm::NavFsm(const NavConfig& cfg)
    : cfg_(cfg) {}

void NavFsm::setConfig(const NavConfig& cfg) {
  cfg_ = cfg;
}

NavState NavFsm::state() const {
  return state_;
}

void NavFsm::reset() {
  state_          = NavState::IDLE;
  have_goal_      = false;
  goal_           = {};
  goal_start_time_  = 0.0;
  state_entry_time_ = 0.0;
  no_path_since_    = -1.0;
  retry_entry_time_   = 0.0;
  last_attempt_time_ = 0.0;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool NavFsm::goalTimedOut(double now) const {
  // goal_timeout <= 0 disables the per-goal timeout (contract: > 0 enables).
  if (cfg_.goal_timeout <= 0.0) return false;
  return (now - goal_start_time_) > cfg_.goal_timeout;
}

NavDecision NavFsm::makeTerminal(NavState terminal_state, NavOutcome outcome) {
  NavDecision d;
  d.next_state    = terminal_state;
  d.mux           = MuxMode::ZERO;
  d.finish_action = true;
  d.outcome       = outcome;
  d.disable_local = true;
  if (cfg_.auto_arm) d.halt = true;
  // Clear active goal.
  have_goal_ = false;
  goal_      = {};
  state_     = terminal_state;
  return d;
}

NavDecision NavFsm::makeIdle() {
  NavDecision d;
  d.next_state = NavState::IDLE;
  d.mux        = MuxMode::ZERO;
  return d;
}

NavDecision NavFsm::enterRetry(double now) {
  // retry_entry_time_ re-anchors on EVERY entry, so retry_budget bounds one episode of
  // CONTINUOUS block, not the whole goal. A planner that oscillates TRACKING<->STUCK
  // could re-enter RETRY (RETRY->TRACK->RECOVER->RETRY) and keep resetting this clock;
  // the backstop is goal_timeout, which the active phases (TRACK/REPLAN/RECOVER) still
  // honor on the never-reset goal_start_time_ (set goal_timeout > 0 to bound that case).
  state_             = NavState::RETRY;
  state_entry_time_  = now;
  retry_entry_time_   = now;
  last_attempt_time_ = now;
  no_path_since_     = -1.0;
  NavDecision d;
  d.next_state     = NavState::RETRY;
  d.mux            = MuxMode::ZERO;   // hold position
  d.forward_goal   = true;           // immediate first re-plan attempt on entry
  d.enable_local   = true;           // keep the local planner ready to pick up a fresh path
  d.reset_watchdog = true;           // not moving -> do not let the watchdog fire here
  return d;
}

// ---------------------------------------------------------------------------
// Main transition function
// ---------------------------------------------------------------------------

NavDecision NavFsm::step(const Observations& obs, double now) {
  // ------------------------------------------------------------------
  // Priority 1: cancel (skip if new_goal is also set -- new_goal supersedes).
  // ------------------------------------------------------------------
  if (obs.cancel && !obs.new_goal) {
    NavDecision d;
    d.next_state    = NavState::IDLE;
    d.mux           = MuxMode::ZERO;
    d.disable_local = true;
    d.finish_action = true;
    d.outcome       = NavOutcome::PREEMPTED;
    if (cfg_.auto_arm) d.halt = true;
    // Clear goal.
    have_goal_ = false;
    goal_      = {};
    no_path_since_ = -1.0;
    state_     = NavState::IDLE;
    return d;
  }

  // ------------------------------------------------------------------
  // Priority 2: new_goal (preempts any current state).
  // ------------------------------------------------------------------
  if (obs.new_goal) {
    goal_           = obs.goal;
    have_goal_      = true;
    goal_start_time_ = now;
    state_entry_time_ = now;
    no_path_since_    = -1.0;

    NavDecision d;
    d.next_state    = NavState::PLAN;
    d.mux           = MuxMode::ZERO;
    d.forward_goal  = true;
    d.enable_local  = true;
    d.reset_watchdog = true;
    if (cfg_.auto_arm) d.arm = true;
    state_ = NavState::PLAN;
    return d;
  }

  // ------------------------------------------------------------------
  // Priority 3: per-state logic.
  // ------------------------------------------------------------------
  switch (state_) {

    // ----------------------------------------------------------------
    case NavState::IDLE: {
      NavDecision d = makeIdle();
      return d;
    }

    // ----------------------------------------------------------------
    case NavState::PLAN: {
      if (obs.local_state == LocalState::TRACKING) {
        state_entry_time_ = now;
        state_            = NavState::TRACK;
        NavDecision d;
        d.next_state     = NavState::TRACK;
        d.mux            = MuxMode::RELAY_TRACK;
        d.reset_watchdog = true;
        return d;
      }
      if (obs.local_state == LocalState::GOAL_REACHED) {
        return makeTerminal(NavState::SUCCEEDED, NavOutcome::SUCCEEDED);
      }
      if ((now - state_entry_time_) > cfg_.plan_timeout) {
        // Soft failure (no track within plan_timeout) -> RETRY: hold + replan.
        return enterRetry(now);
      }
      if (goalTimedOut(now)) {
        return makeTerminal(NavState::ABORTED, NavOutcome::ABORTED_TIMEOUT);
      }
      // Stay in PLAN.
      NavDecision d;
      d.next_state = NavState::PLAN;
      d.mux        = MuxMode::ZERO;
      return d;
    }

    // ----------------------------------------------------------------
    case NavState::TRACK: {
      if (obs.local_state == LocalState::GOAL_REACHED) {
        return makeTerminal(NavState::SUCCEEDED, NavOutcome::SUCCEEDED);
      }
      // Soft failures (local planner STUCK, or watchdog no_progress during a slow
      // near-goal approach) -> RETRY: hold position + re-forward goal. No blind
      // recovery motion. enterRetry resets the watchdog so the hold does not re-fire.
      if (obs.local_state == LocalState::STUCK) {
        return enterRetry(now);
      }
      if (obs.no_progress) {
        return enterRetry(now);
      }
      if (obs.local_state == LocalState::NO_PATH) {
        if (no_path_since_ < 0.0) {
          no_path_since_ = now;
        }
        if ((now - no_path_since_) > cfg_.no_path_grace) {
          // Debounce expired: go to REPLAN.
          no_path_since_    = -1.0;
          state_entry_time_ = now;
          state_            = NavState::REPLAN;
          NavDecision d;
          d.next_state   = NavState::REPLAN;
          d.mux          = MuxMode::ZERO;
          d.forward_goal = true;
          return d;
        }
        // Still within grace: stay TRACK, relay.
        NavDecision d;
        d.next_state = NavState::TRACK;
        d.mux        = MuxMode::RELAY_TRACK;
        return d;
      }
      // Clear debounce when local returns to TRACKING.
      if (obs.local_state == LocalState::TRACKING) {
        no_path_since_ = -1.0;
      }
      if (goalTimedOut(now)) {
        return makeTerminal(NavState::ABORTED, NavOutcome::ABORTED_TIMEOUT);
      }
      // Stay TRACK.
      NavDecision d;
      d.next_state = NavState::TRACK;
      d.mux        = MuxMode::RELAY_TRACK;
      return d;
    }

    // ----------------------------------------------------------------
    case NavState::REPLAN: {
      if (obs.local_state == LocalState::TRACKING) {
        state_entry_time_ = now;
        state_            = NavState::TRACK;
        NavDecision d;
        d.next_state     = NavState::TRACK;
        d.mux            = MuxMode::RELAY_TRACK;
        d.reset_watchdog = true;
        return d;
      }
      if ((now - state_entry_time_) > cfg_.replan_timeout) {
        // Soft failure (no fresh path within replan_timeout) -> RETRY: hold + replan.
        return enterRetry(now);
      }
      if (goalTimedOut(now)) {
        return makeTerminal(NavState::ABORTED, NavOutcome::ABORTED_TIMEOUT);
      }
      // Stay REPLAN.
      NavDecision d;
      d.next_state = NavState::REPLAN;
      d.mux        = MuxMode::ZERO;
      return d;
    }

    // ----------------------------------------------------------------
    // RECOVER removed: soft failures route to RETRY (above + below). NavState::RECOVER
    // remains a reserved/never-emitted wire constant; it falls through to default.

    // ----------------------------------------------------------------
    // RETRY: blocked goal. Hold position and re-forward the goal on a
    // cadence within retry_budget; resume on a fresh path; abort only on timeout.
    // goal_timeout is NOT checked here -- retry_budget is the authority for the
    // blocked phase.
    case NavState::RETRY: {
      if (obs.local_state == LocalState::GOAL_REACHED) {
        return makeTerminal(NavState::SUCCEEDED, NavOutcome::SUCCEEDED);
      }
      if (obs.local_state == LocalState::TRACKING) {
        state_entry_time_ = now;
        state_            = NavState::TRACK;
        NavDecision d;
        d.next_state     = NavState::TRACK;
        d.mux            = MuxMode::RELAY_TRACK;
        d.reset_watchdog = true;
        return d;
      }
      const RetryAction act = decideRetryAction(
          now - retry_entry_time_, now - last_attempt_time_,
          cfg_.retry_budget, cfg_.retry_replan_interval);
      if (act == RetryAction::TIMEOUT) {
        return makeTerminal(NavState::ABORTED, NavOutcome::ABORTED_BLOCKED_TIMEOUT);
      }
      NavDecision d;
      d.next_state   = NavState::RETRY;
      d.mux          = MuxMode::ZERO;
      d.enable_local = true;
      if (act == RetryAction::ATTEMPT) {
        d.forward_goal     = true;
        last_attempt_time_ = now;
      }
      return d;
    }

    // ----------------------------------------------------------------
    // SUCCEEDED and ABORTED are one-tick transient states.
    // On the next step() with no new_goal/cancel they collapse to IDLE.
    case NavState::SUCCEEDED:
    case NavState::ABORTED: {
      state_ = NavState::IDLE;
      return makeIdle();
    }

    default:
      // Should never happen; reset to IDLE defensively.
      state_ = NavState::IDLE;
      return makeIdle();
  }
}

} // namespace g1_nav
