// test_fsm.cpp -- gtest coverage of every NavFsm transition in core-interface.md.
// ROS-free; links only g1_nav_core.

#include <gtest/gtest.h>
#include "g1_nav/core/fsm.h"

using namespace g1_nav;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static NavConfig defaultCfg() {
  NavConfig c;
  c.plan_timeout   = 5.0;
  c.replan_timeout = 3.0;
  c.goal_timeout   = 120.0;
  c.no_path_grace  = 1.0;
  c.auto_arm       = true;
  return c;
}

// Config with the blocked-goal RETRY enabled (production default).
static NavConfig retryCfg() {
  NavConfig c = defaultCfg();
  c.retry_budget          = 120.0;
  c.retry_replan_interval = 1.0;
  return c;
}

static Observations idle_obs() {
  Observations o;
  o.local_state = LocalState::NO_GOAL;
  return o;
}

static Observations new_goal_obs(Pose2D goal = {}) {
  Observations o;
  o.new_goal    = true;
  o.goal        = goal;
  o.local_state = LocalState::NO_GOAL;
  return o;
}

// ---------------------------------------------------------------------------
// IDLE + new_goal -> PLAN
// ---------------------------------------------------------------------------

TEST(NavFsm, IdleNewGoal_ToPlan_AutoArmTrue) {
  NavConfig cfg = defaultCfg();
  cfg.auto_arm  = true;
  NavFsm fsm(cfg);
  ASSERT_EQ(fsm.state(), NavState::IDLE);

  Pose2D g; g.x = 1.0; g.y = 2.0; g.yaw = 0.5;
  NavDecision d = fsm.step(new_goal_obs(g), 0.0);

  EXPECT_EQ(d.next_state, NavState::PLAN);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_TRUE(d.forward_goal);
  EXPECT_TRUE(d.enable_local);
  EXPECT_TRUE(d.reset_watchdog);
  EXPECT_TRUE(d.arm);       // auto_arm = true
  EXPECT_FALSE(d.halt);
  EXPECT_FALSE(d.finish_action);
  EXPECT_EQ(fsm.state(), NavState::PLAN);
}

TEST(NavFsm, IdleNewGoal_ToPlan_AutoArmFalse) {
  NavConfig cfg = defaultCfg();
  cfg.auto_arm  = false;
  NavFsm fsm(cfg);

  NavDecision d = fsm.step(new_goal_obs(), 0.0);

  EXPECT_EQ(d.next_state, NavState::PLAN);
  EXPECT_FALSE(d.arm);   // auto_arm = false -> no arm
  EXPECT_FALSE(d.halt);
}

// ---------------------------------------------------------------------------
// PLAN + TRACKING -> TRACK
// ---------------------------------------------------------------------------

TEST(NavFsm, Plan_LocalTracking_ToTrack) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);

  Observations o;
  o.local_state = LocalState::TRACKING;
  NavDecision d = fsm.step(o, 0.1);

  EXPECT_EQ(d.next_state, NavState::TRACK);
  EXPECT_EQ(d.mux, MuxMode::RELAY_TRACK);
  EXPECT_TRUE(d.reset_watchdog);
  EXPECT_FALSE(d.forward_goal);
  EXPECT_FALSE(d.finish_action);
  EXPECT_EQ(fsm.state(), NavState::TRACK);
}

// ---------------------------------------------------------------------------
// PLAN + GOAL_REACHED -> SUCCEEDED (terminal)
// ---------------------------------------------------------------------------

TEST(NavFsm, Plan_GoalReached_ToSucceeded) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);

  Observations o;
  o.local_state = LocalState::GOAL_REACHED;
  NavDecision d = fsm.step(o, 0.1);

  EXPECT_EQ(d.next_state, NavState::SUCCEEDED);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(d.outcome, NavOutcome::SUCCEEDED);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_TRUE(d.halt);          // auto_arm = true
  EXPECT_TRUE(d.disable_local);
  EXPECT_EQ(fsm.state(), NavState::SUCCEEDED);
}

// ---------------------------------------------------------------------------
// PLAN + goal_timeout -> ABORTED_TIMEOUT
// ---------------------------------------------------------------------------

TEST(NavFsm, Plan_GoalTimeout_ToAborted) {
  NavConfig cfg = defaultCfg();
  cfg.plan_timeout = 200.0;  // ensure plan_timeout does not fire first
  cfg.goal_timeout = 10.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  Observations o;
  o.local_state = LocalState::NO_GOAL;
  NavDecision d = fsm.step(o, 10.1);

  EXPECT_EQ(d.next_state, NavState::ABORTED);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(d.outcome, NavOutcome::ABORTED_TIMEOUT);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_EQ(fsm.state(), NavState::ABORTED);
}

// ---------------------------------------------------------------------------
// TRACK + GOAL_REACHED -> SUCCEEDED
// ---------------------------------------------------------------------------

TEST(NavFsm, Track_GoalReached_ToSucceeded) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking;
  tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);  // -> TRACK

  ASSERT_EQ(fsm.state(), NavState::TRACK);

  Observations reached;
  reached.local_state = LocalState::GOAL_REACHED;
  NavDecision d = fsm.step(reached, 0.2);

  EXPECT_EQ(d.next_state, NavState::SUCCEEDED);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(d.outcome, NavOutcome::SUCCEEDED);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_TRUE(d.halt);  // auto_arm = true
}

// ---------------------------------------------------------------------------
// TRACK + STUCK -> RETRY (soft failure: hold + replan, no blind recovery)
// ---------------------------------------------------------------------------

TEST(NavFsm, Track_Stuck_ToRetry) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);

  Observations stuck; stuck.local_state = LocalState::STUCK;
  NavDecision d = fsm.step(stuck, 0.2);

  EXPECT_EQ(d.next_state, NavState::RETRY);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_TRUE(d.forward_goal);
  EXPECT_TRUE(d.enable_local);
  EXPECT_TRUE(d.reset_watchdog);
  EXPECT_FALSE(d.finish_action);
  EXPECT_EQ(fsm.state(), NavState::RETRY);
}

// ---------------------------------------------------------------------------
// TRACK + no_progress -> RETRY (soft failure: hold + replan, no blind recovery)
// ---------------------------------------------------------------------------

TEST(NavFsm, Track_NoProgress_ToRetry) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);

  Observations obs; obs.local_state = LocalState::TRACKING; obs.no_progress = true;
  NavDecision d = fsm.step(obs, 0.2);

  EXPECT_EQ(d.next_state, NavState::RETRY);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_TRUE(d.forward_goal);
  EXPECT_TRUE(d.enable_local);
  EXPECT_TRUE(d.reset_watchdog);
  EXPECT_EQ(fsm.state(), NavState::RETRY);
}

// ---------------------------------------------------------------------------
// TRACK + NO_PATH debounce: stays TRACK during grace, REPLAN after
// ---------------------------------------------------------------------------

TEST(NavFsm, Track_NoPath_Debounce_ThenReplan) {
  NavConfig cfg = defaultCfg();
  cfg.no_path_grace = 1.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);  // -> TRACK

  // Within grace: should stay TRACK.
  Observations no_path; no_path.local_state = LocalState::NO_PATH;
  NavDecision d1 = fsm.step(no_path, 0.2);
  EXPECT_EQ(d1.next_state, NavState::TRACK);
  EXPECT_EQ(d1.mux, MuxMode::RELAY_TRACK);
  EXPECT_EQ(fsm.state(), NavState::TRACK);

  // At grace boundary (not yet exceeded).
  NavDecision d2 = fsm.step(no_path, 1.1);
  EXPECT_EQ(d2.next_state, NavState::TRACK);

  // After grace: REPLAN.
  NavDecision d3 = fsm.step(no_path, 1.21);
  EXPECT_EQ(d3.next_state, NavState::REPLAN);
  EXPECT_EQ(d3.mux, MuxMode::ZERO);
  EXPECT_TRUE(d3.forward_goal);  // re-nudge the planner
  EXPECT_EQ(fsm.state(), NavState::REPLAN);
}

// ---------------------------------------------------------------------------
// TRACK + NO_PATH then back to TRACKING clears debounce (no spurious REPLAN)
// ---------------------------------------------------------------------------

TEST(NavFsm, Track_NoPath_ClearedByTracking_NoSpuriousReplan) {
  NavConfig cfg = defaultCfg();
  cfg.no_path_grace = 1.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);  // -> TRACK

  // NO_PATH for half the grace.
  Observations no_path; no_path.local_state = LocalState::NO_PATH;
  fsm.step(no_path, 0.2);
  fsm.step(no_path, 0.5);

  // Back to TRACKING (clears debounce).
  NavDecision d = fsm.step(tracking, 0.6);
  EXPECT_EQ(d.next_state, NavState::TRACK);
  EXPECT_EQ(fsm.state(), NavState::TRACK);

  // More TRACKING ticks past the old grace deadline: should NOT REPLAN.
  NavDecision d2 = fsm.step(tracking, 1.5);
  EXPECT_EQ(d2.next_state, NavState::TRACK);
  EXPECT_EQ(fsm.state(), NavState::TRACK);
}

// ---------------------------------------------------------------------------
// TRACK + goal_timeout -> ABORTED_TIMEOUT
// ---------------------------------------------------------------------------

TEST(NavFsm, Track_GoalTimeout_ToAborted) {
  NavConfig cfg = defaultCfg();
  cfg.goal_timeout = 10.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);

  Observations obs; obs.local_state = LocalState::TRACKING;
  NavDecision d = fsm.step(obs, 10.1);

  EXPECT_EQ(d.next_state, NavState::ABORTED);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(d.outcome, NavOutcome::ABORTED_TIMEOUT);
}

// ---------------------------------------------------------------------------
// REPLAN + TRACKING -> TRACK (with reset_watchdog)
// ---------------------------------------------------------------------------

TEST(NavFsm, Replan_Tracking_ToTrack) {
  NavConfig cfg = defaultCfg();
  cfg.no_path_grace = 0.5;  // short grace to make REPLAN easy to trigger
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);  // -> TRACK

  // Start NO_PATH debounce at t=0.2.
  Observations no_path; no_path.local_state = LocalState::NO_PATH;
  NavDecision d_grace = fsm.step(no_path, 0.2);  // within grace, stays TRACK
  EXPECT_EQ(d_grace.next_state, NavState::TRACK);

  // After grace has elapsed: REPLAN.
  NavDecision d1 = fsm.step(no_path, 0.75);
  EXPECT_EQ(d1.next_state, NavState::REPLAN);
  EXPECT_EQ(fsm.state(), NavState::REPLAN);

  // Now local becomes TRACKING -> back to TRACK.
  NavDecision d2 = fsm.step(tracking, 0.8);
  EXPECT_EQ(d2.next_state, NavState::TRACK);
  EXPECT_EQ(d2.mux, MuxMode::RELAY_TRACK);
  EXPECT_TRUE(d2.reset_watchdog);
  EXPECT_EQ(fsm.state(), NavState::TRACK);
}

// ---------------------------------------------------------------------------
// REPLAN + goal_timeout -> ABORTED_TIMEOUT
// ---------------------------------------------------------------------------

TEST(NavFsm, Replan_GoalTimeout_ToAborted) {
  NavConfig cfg = defaultCfg();
  cfg.no_path_grace  = 0.5;
  cfg.replan_timeout = 200.0;
  cfg.goal_timeout   = 5.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);

  // Debounce: start at t=0.2, grace=0.5 expires at t=0.7.
  Observations no_path; no_path.local_state = LocalState::NO_PATH;
  fsm.step(no_path, 0.2);
  fsm.step(no_path, 0.75);  // -> REPLAN

  ASSERT_EQ(fsm.state(), NavState::REPLAN);

  Observations no_goal_obs; no_goal_obs.local_state = LocalState::NO_GOAL;
  // goal_timeout=5.0 from t=0: fires when now > 5.0
  NavDecision d = fsm.step(no_goal_obs, 5.1);

  EXPECT_EQ(d.next_state, NavState::ABORTED);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(d.outcome, NavOutcome::ABORTED_TIMEOUT);
}

// ---------------------------------------------------------------------------
// Cancel from TRACK -> IDLE (PREEMPTED, halt)
// ---------------------------------------------------------------------------

TEST(NavFsm, Track_Cancel_ToIdle) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);
  ASSERT_EQ(fsm.state(), NavState::TRACK);

  Observations cancel; cancel.cancel = true; cancel.local_state = LocalState::TRACKING;
  NavDecision d = fsm.step(cancel, 0.2);

  EXPECT_EQ(d.next_state, NavState::IDLE);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_TRUE(d.disable_local);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(d.outcome, NavOutcome::PREEMPTED);
  EXPECT_TRUE(d.halt);  // auto_arm = true
  EXPECT_EQ(fsm.state(), NavState::IDLE);
}

// ---------------------------------------------------------------------------
// New goal preempts mid-TRACK (goal_start_time reset; new goal forwarded)
// ---------------------------------------------------------------------------

TEST(NavFsm, Track_NewGoalPreempts) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);
  ASSERT_EQ(fsm.state(), NavState::TRACK);

  Pose2D new_g; new_g.x = 5.0; new_g.y = 5.0;
  Observations ng; ng.new_goal = true; ng.goal = new_g;
  ng.local_state = LocalState::TRACKING;
  NavDecision d = fsm.step(ng, 1.0);

  EXPECT_EQ(d.next_state, NavState::PLAN);
  EXPECT_TRUE(d.forward_goal);
  EXPECT_TRUE(d.enable_local);
  EXPECT_TRUE(d.reset_watchdog);
  EXPECT_TRUE(d.arm);  // auto_arm=true
  EXPECT_FALSE(d.finish_action);  // FSM just switches; node preempts the old handle
  EXPECT_EQ(fsm.state(), NavState::PLAN);
}

// ---------------------------------------------------------------------------
// New goal supersedes a simultaneous cancel (cancel is ignored)
// ---------------------------------------------------------------------------

TEST(NavFsm, NewGoalSupersedesCancel) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);

  Observations obs;
  obs.new_goal  = true;
  obs.cancel    = true;  // both set -- new_goal wins
  obs.goal      = {};
  obs.local_state = LocalState::TRACKING;
  NavDecision d = fsm.step(obs, 0.1);

  EXPECT_EQ(d.next_state, NavState::PLAN);
  EXPECT_FALSE(d.finish_action);  // not a cancel outcome
  EXPECT_TRUE(d.forward_goal);
  EXPECT_EQ(fsm.state(), NavState::PLAN);
}

// ---------------------------------------------------------------------------
// Terminal one-tick transient: SUCCEEDED -> IDLE on next step (no intents)
// ---------------------------------------------------------------------------

TEST(NavFsm, Succeeded_NextTick_ToIdle) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);

  Observations reached; reached.local_state = LocalState::GOAL_REACHED;
  fsm.step(reached, 0.1);  // -> SUCCEEDED (via PLAN shortcut)

  ASSERT_EQ(fsm.state(), NavState::SUCCEEDED);

  // Next tick with empty obs -> IDLE, no intents.
  NavDecision d = fsm.step(idle_obs(), 0.2);
  EXPECT_EQ(d.next_state, NavState::IDLE);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_FALSE(d.forward_goal);
  EXPECT_FALSE(d.enable_local);
  EXPECT_FALSE(d.disable_local);
  EXPECT_FALSE(d.arm);
  EXPECT_FALSE(d.halt);
  EXPECT_FALSE(d.finish_action);
  EXPECT_EQ(fsm.state(), NavState::IDLE);
}

TEST(NavFsm, Aborted_NextTick_ToIdle) {
  NavConfig cfg = defaultCfg();
  cfg.plan_timeout = 200.0;  // high so it does not fire first
  cfg.goal_timeout = 0.5;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  // Trigger goal_timeout -> ABORTED.
  Observations no_goal_obs; no_goal_obs.local_state = LocalState::NO_GOAL;
  fsm.step(no_goal_obs, 0.6);  // -> ABORTED
  ASSERT_EQ(fsm.state(), NavState::ABORTED);

  // Next tick -> IDLE, no intents.
  NavDecision d = fsm.step(idle_obs(), 0.8);
  EXPECT_EQ(d.next_state, NavState::IDLE);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_FALSE(d.finish_action);
  EXPECT_FALSE(d.arm);
  EXPECT_FALSE(d.halt);
  EXPECT_EQ(fsm.state(), NavState::IDLE);
}

// ---------------------------------------------------------------------------
// auto_arm=false: full accept -> track -> terminal emits NO arm and NO halt
// ---------------------------------------------------------------------------

TEST(NavFsm, AutoArmFalse_NoArmNoHalt) {
  NavConfig cfg = defaultCfg();
  cfg.auto_arm = false;
  NavFsm fsm(cfg);

  NavDecision d_plan = fsm.step(new_goal_obs(), 0.0);
  EXPECT_FALSE(d_plan.arm);
  EXPECT_FALSE(d_plan.halt);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  NavDecision d_track = fsm.step(tracking, 0.1);
  EXPECT_FALSE(d_track.arm);
  EXPECT_FALSE(d_track.halt);

  Observations reached; reached.local_state = LocalState::GOAL_REACHED;
  NavDecision d_done = fsm.step(reached, 0.2);
  EXPECT_EQ(d_done.next_state, NavState::SUCCEEDED);
  EXPECT_FALSE(d_done.arm);
  EXPECT_FALSE(d_done.halt);
}

// ---------------------------------------------------------------------------
// goal_timeout = 0 means unbounded (never fires)
// ---------------------------------------------------------------------------

TEST(NavFsm, GoalTimeout_Zero_Unbounded) {
  NavConfig cfg = defaultCfg();
  cfg.goal_timeout = 0.0;
  cfg.plan_timeout = 9999.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);

  // Tick far into the future: should stay TRACK, not ABORTED.
  Observations obs; obs.local_state = LocalState::TRACKING;
  NavDecision d = fsm.step(obs, 1000000.0);
  EXPECT_EQ(d.next_state, NavState::TRACK);
  EXPECT_FALSE(d.finish_action);
}

// ---------------------------------------------------------------------------
// reset() returns FSM to IDLE regardless of current state
// ---------------------------------------------------------------------------

TEST(NavFsm, Reset_ForcesIdle) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);
  ASSERT_EQ(fsm.state(), NavState::PLAN);

  fsm.reset();
  EXPECT_EQ(fsm.state(), NavState::IDLE);

  // Post-reset: a plain tick with empty obs stays IDLE.
  NavDecision d = fsm.step(idle_obs(), 0.1);
  EXPECT_EQ(d.next_state, NavState::IDLE);
}

// ---------------------------------------------------------------------------
// setConfig is live: the next step() uses the new config.
// ---------------------------------------------------------------------------

TEST(NavFsm, SetConfig_LiveUpdate) {
  NavConfig cfg = defaultCfg();
  cfg.plan_timeout = 100.0;  // would not timeout soon
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  NavConfig cfg2 = cfg;
  cfg2.plan_timeout = 0.5;
  fsm.setConfig(cfg2);

  Observations no_goal_obs; no_goal_obs.local_state = LocalState::NO_GOAL;
  NavDecision d = fsm.step(no_goal_obs, 0.6);  // should now timeout
  EXPECT_EQ(d.next_state, NavState::RETRY);
}

// ---------------------------------------------------------------------------
// IDLE stays IDLE with repeated empty ticks (no spurious transitions)
// ---------------------------------------------------------------------------

TEST(NavFsm, Idle_StaysIdle) {
  NavFsm fsm(defaultCfg());
  for (int i = 0; i < 20; ++i) {
    NavDecision d = fsm.step(idle_obs(), static_cast<double>(i));
    EXPECT_EQ(d.next_state, NavState::IDLE);
    EXPECT_EQ(d.mux, MuxMode::ZERO);
    EXPECT_FALSE(d.finish_action);
  }
  EXPECT_EQ(fsm.state(), NavState::IDLE);
}

// ---------------------------------------------------------------------------
// M3: goal_timeout=0 is unbounded in PLAN and REPLAN
// ---------------------------------------------------------------------------

TEST(NavFsm, GoalTimeout_Zero_Unbounded_InPlan) {
  NavConfig cfg = defaultCfg();
  cfg.goal_timeout = 0.0;
  cfg.plan_timeout = 1e9;  // will not fire during this test
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);
  ASSERT_EQ(fsm.state(), NavState::PLAN);

  // Tick far into the future (but within plan_timeout) while still in PLAN:
  // must NOT emit ABORTED_TIMEOUT.
  Observations obs; obs.local_state = LocalState::NO_GOAL;
  NavDecision d = fsm.step(obs, 1000000.0);
  EXPECT_EQ(d.next_state, NavState::PLAN);
  EXPECT_NE(d.outcome, NavOutcome::ABORTED_TIMEOUT);
  EXPECT_FALSE(d.finish_action);
}

TEST(NavFsm, GoalTimeout_Zero_Unbounded_InReplan) {
  NavConfig cfg = defaultCfg();
  cfg.goal_timeout   = 0.0;
  cfg.no_path_grace  = 0.5;
  cfg.replan_timeout = 1e9;  // will not fire during this test
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);

  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);  // -> TRACK

  Observations no_path; no_path.local_state = LocalState::NO_PATH;
  fsm.step(no_path, 0.2);   // debounce start
  fsm.step(no_path, 0.75);  // -> REPLAN
  ASSERT_EQ(fsm.state(), NavState::REPLAN);

  // Far-future tick in REPLAN (still within replan_timeout=1e9):
  // must NOT emit ABORTED_TIMEOUT.
  Observations obs; obs.local_state = LocalState::NO_GOAL;
  NavDecision d = fsm.step(obs, 1000000.0);
  EXPECT_EQ(d.next_state, NavState::REPLAN);
  EXPECT_NE(d.outcome, NavOutcome::ABORTED_TIMEOUT);
  EXPECT_FALSE(d.finish_action);
}

// ---------------------------------------------------------------------------
// M4: cancel from IDLE and cancel from PLAN -> IDLE with PREEMPTED + halt
// ---------------------------------------------------------------------------

TEST(NavFsm, Cancel_FromIdle_ToIdle_Preempted) {
  NavFsm fsm(defaultCfg());
  ASSERT_EQ(fsm.state(), NavState::IDLE);

  Observations cancel; cancel.cancel = true; cancel.local_state = LocalState::NO_GOAL;
  NavDecision d = fsm.step(cancel, 0.0);

  // Cancel with no active goal: still produces IDLE + finish_action + PREEMPTED + halt.
  EXPECT_EQ(d.next_state, NavState::IDLE);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(d.outcome, NavOutcome::PREEMPTED);
  EXPECT_TRUE(d.halt);  // auto_arm = true
  EXPECT_EQ(fsm.state(), NavState::IDLE);
}

TEST(NavFsm, Cancel_FromPlan_ToIdle_Preempted) {
  NavFsm fsm(defaultCfg());
  fsm.step(new_goal_obs(), 0.0);
  ASSERT_EQ(fsm.state(), NavState::PLAN);

  Observations cancel; cancel.cancel = true; cancel.local_state = LocalState::NO_GOAL;
  NavDecision d = fsm.step(cancel, 0.1);

  EXPECT_EQ(d.next_state, NavState::IDLE);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(d.outcome, NavOutcome::PREEMPTED);
  EXPECT_TRUE(d.halt);  // auto_arm = true
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_EQ(fsm.state(), NavState::IDLE);
}

// ---------------------------------------------------------------------------
// I1 sentinel regression: time origin at 0.0 must not be mistaken for inactive
// ---------------------------------------------------------------------------

TEST(NavFsm, NoPathSentinel_TimeOriginAtZero) {
  // This test must FAIL with the old 0.0 sentinel (the first NO_PATH at now=0.0
  // would match the "not active" value and always re-stamp no_path_since_=0.0,
  // making the debounce perpetually restart -- transition to REPLAN never fires).
  // With the -1.0 sentinel it must PASS.
  NavConfig cfg = defaultCfg();
  cfg.no_path_grace = 1.0;
  NavFsm fsm(cfg);

  // Accept goal at now=0.0 and enter TRACK at now=0.0 (both on the time origin).
  fsm.step(new_goal_obs(), 0.0);
  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.0);  // PLAN->TRACK at time 0.0
  ASSERT_EQ(fsm.state(), NavState::TRACK);

  // Feed NO_PATH starting at now=0.0 (the time origin).
  Observations no_path; no_path.local_state = LocalState::NO_PATH;

  // First NO_PATH tick at now=0.0: debounce starts; within grace -> stay TRACK.
  NavDecision d0 = fsm.step(no_path, 0.0);
  EXPECT_EQ(d0.next_state, NavState::TRACK)
      << "Debounce should START at now=0.0 but still be within grace";

  // Within grace at now=0.5 (0.5 < 1.0 grace): stay TRACK.
  NavDecision d1 = fsm.step(no_path, 0.5);
  EXPECT_EQ(d1.next_state, NavState::TRACK)
      << "Within grace at now=0.5: should remain TRACK";

  // Past grace at now=1.1 (1.1 - 0.0 > 1.0): must transition to REPLAN.
  NavDecision d2 = fsm.step(no_path, 1.1);
  EXPECT_EQ(d2.next_state, NavState::REPLAN)
      << "After grace (now-0.0 > 1.0): must go to REPLAN";
  EXPECT_TRUE(d2.forward_goal);
  EXPECT_EQ(fsm.state(), NavState::REPLAN);
}

// ===========================================================================
// RETRY: blocked-goal hold + re-plan (the sole soft-failure handler).
// ===========================================================================

// PLAN timeout -> RETRY.
TEST(NavFsm, Plan_PlanTimeout_ToRetry) {
  NavConfig cfg = retryCfg();
  cfg.plan_timeout = 2.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);  // -> PLAN

  Observations o; o.local_state = LocalState::NO_PATH;
  NavDecision d = fsm.step(o, 2.1);  // plan_timeout exceeded

  EXPECT_EQ(d.next_state, NavState::RETRY);
  EXPECT_EQ(d.mux, MuxMode::ZERO);
  EXPECT_TRUE(d.forward_goal);    // immediate first re-plan attempt on entry
  EXPECT_TRUE(d.enable_local);
  EXPECT_TRUE(d.reset_watchdog);
  EXPECT_FALSE(d.finish_action);
  EXPECT_EQ(fsm.state(), NavState::RETRY);
}

// RETRY resumes tracking the moment a fresh path appears.
TEST(NavFsm, Retry_ResumesOnTracking) {
  NavConfig cfg = retryCfg();
  cfg.plan_timeout = 1.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);
  Observations o; o.local_state = LocalState::NO_PATH;
  fsm.step(o, 2.0);  // -> RETRY
  ASSERT_EQ(fsm.state(), NavState::RETRY);

  Observations t; t.local_state = LocalState::TRACKING;
  NavDecision d = fsm.step(t, 2.5);
  EXPECT_EQ(d.next_state, NavState::TRACK);
  EXPECT_EQ(d.mux, MuxMode::RELAY_TRACK);
  EXPECT_TRUE(d.reset_watchdog);
  EXPECT_EQ(fsm.state(), NavState::TRACK);
}

// RETRY holds below the interval and re-forwards the goal at the interval.
TEST(NavFsm, Retry_HoldThenAttemptOnInterval) {
  NavConfig cfg = retryCfg();
  cfg.plan_timeout = 1.0;
  cfg.retry_replan_interval = 1.0;
  cfg.retry_budget = 120.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);
  Observations o; o.local_state = LocalState::NO_PATH;
  fsm.step(o, 2.0);  // enter RETRY at t=2 (last attempt at t=2)

  NavDecision hold = fsm.step(o, 2.4);  // 0.4 s since last attempt -> HOLD
  EXPECT_EQ(hold.next_state, NavState::RETRY);
  EXPECT_FALSE(hold.forward_goal);

  NavDecision attempt = fsm.step(o, 3.1);  // 1.1 s since last attempt -> ATTEMPT
  EXPECT_EQ(attempt.next_state, NavState::RETRY);
  EXPECT_TRUE(attempt.forward_goal);
  EXPECT_TRUE(attempt.enable_local);
}

// RETRY aborts with ABORTED_BLOCKED_TIMEOUT only after the budget elapses.
TEST(NavFsm, Retry_TimesOut_ToBlockedAbort) {
  NavConfig cfg = retryCfg();
  cfg.plan_timeout = 1.0;
  cfg.retry_budget = 120.0;
  cfg.goal_timeout = 0.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);
  Observations o; o.local_state = LocalState::NO_PATH;
  fsm.step(o, 2.0);  // -> RETRY at t=2

  NavDecision d = fsm.step(o, 2.0 + 120.1);  // budget exceeded
  EXPECT_EQ(d.next_state, NavState::ABORTED);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(d.outcome, NavOutcome::ABORTED_BLOCKED_TIMEOUT);
  EXPECT_TRUE(d.halt);          // auto_arm
  EXPECT_TRUE(d.disable_local);
  EXPECT_EQ(fsm.state(), NavState::ABORTED);
}

// RETRY ignores goal_timeout: the RETRY phase is bounded ONLY by retry_budget,
// so a small goal_timeout cannot cut RETRY short.
TEST(NavFsm, Retry_IgnoresGoalTimeout) {
  NavConfig cfg = retryCfg();
  cfg.plan_timeout = 1.0;
  cfg.goal_timeout = 5.0;       // would fire at t=5 in the active states
  cfg.retry_budget = 120.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);
  Observations o; o.local_state = LocalState::NO_PATH;
  fsm.step(o, 2.0);  // -> RETRY at t=2

  NavDecision d = fsm.step(o, 10.0);  // past goal_timeout but within retry_budget
  EXPECT_EQ(d.next_state, NavState::RETRY);
  EXPECT_FALSE(d.finish_action);
}

// REPLAN timeout -> RETRY.
TEST(NavFsm, Replan_ReplanTimeout_ToRetry) {
  NavConfig cfg = retryCfg();
  cfg.no_path_grace  = 0.5;
  cfg.replan_timeout = 2.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);
  Observations tracking; tracking.local_state = LocalState::TRACKING;
  fsm.step(tracking, 0.1);  // -> TRACK
  Observations no_path; no_path.local_state = LocalState::NO_PATH;
  fsm.step(no_path, 0.2);   // debounce start (within grace)
  fsm.step(no_path, 0.75);  // -> REPLAN at t=0.75
  ASSERT_EQ(fsm.state(), NavState::REPLAN);

  Observations no_goal; no_goal.local_state = LocalState::NO_GOAL;
  NavDecision d = fsm.step(no_goal, 2.8);  // now - 0.75 > 2.0
  EXPECT_EQ(d.next_state, NavState::RETRY);
  EXPECT_TRUE(d.forward_goal);
}

// RETRY + GOAL_REACHED -> SUCCEEDED (e.g. the goal was already within tolerance).
TEST(NavFsm, Retry_GoalReached_ToSucceeded) {
  NavConfig cfg = retryCfg();
  cfg.plan_timeout = 1.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);
  Observations o; o.local_state = LocalState::NO_PATH;
  fsm.step(o, 2.0);  // -> RETRY

  Observations reached; reached.local_state = LocalState::GOAL_REACHED;
  NavDecision d = fsm.step(reached, 2.5);
  EXPECT_EQ(d.next_state, NavState::SUCCEEDED);
  EXPECT_EQ(d.outcome, NavOutcome::SUCCEEDED);
  EXPECT_TRUE(d.finish_action);
}

// Cancel preempts RETRY -> IDLE (PREEMPTED).
TEST(NavFsm, Retry_Cancel_ToIdle) {
  NavConfig cfg = retryCfg();
  cfg.plan_timeout = 1.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);
  Observations o; o.local_state = LocalState::NO_PATH;
  fsm.step(o, 2.0);  // -> RETRY

  Observations cancel; cancel.cancel = true; cancel.local_state = LocalState::NO_PATH;
  NavDecision d = fsm.step(cancel, 2.5);
  EXPECT_EQ(d.next_state, NavState::IDLE);
  EXPECT_EQ(d.outcome, NavOutcome::PREEMPTED);
  EXPECT_TRUE(d.finish_action);
  EXPECT_EQ(fsm.state(), NavState::IDLE);
}

// A new goal preempts RETRY -> PLAN.
TEST(NavFsm, Retry_NewGoal_ToPlan) {
  NavConfig cfg = retryCfg();
  cfg.plan_timeout = 1.0;
  NavFsm fsm(cfg);
  fsm.step(new_goal_obs(), 0.0);
  Observations o; o.local_state = LocalState::NO_PATH;
  fsm.step(o, 2.0);  // -> RETRY

  Pose2D g2; g2.x = 9.0; g2.y = 1.0;
  Observations ng; ng.new_goal = true; ng.goal = g2; ng.local_state = LocalState::NO_PATH;
  NavDecision d = fsm.step(ng, 2.5);
  EXPECT_EQ(d.next_state, NavState::PLAN);
  EXPECT_TRUE(d.forward_goal);
  EXPECT_EQ(fsm.state(), NavState::PLAN);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
