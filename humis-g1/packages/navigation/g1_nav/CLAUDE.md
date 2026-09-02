# g1_nav - agent guide

Navigation FSM orchestrator for the Unitree G1. THIN COORDINATOR over g1_costmap,
g1_global_planner, g1_local_planner, and g1_locomotion. Do NOT embed planning,
tracking, or costmap logic here.

## Status
Core: nav_types.h, NavFsm (fsm.*), ProgressWatchdog (progress_watchdog.*),
decideRetryAction (retry.*), gtest cases
(test_fsm incl. RETRY / test_progress_watchdog / test_retry).
Node: g1_nav_node (src/nav_node.cpp) wires the core helpers into the
orchestrator. All ROS scaffolding now EXISTS: config/nav.yaml, cfg/NavTuning.cfg
(dynamic_reconfigure live knobs), launch/nav.launch (opt-in upstream args +
single-writer/sole-goal topic args forwarded to the sub-launches), config/nav.rviz, and the python launch/config
contract test (test/test_nav_launch.py). End-to-end gate: test/nav_integration.test
+ test/nav_integration_test.py -- one event-driven mock node (mock upstreams + action
client + /nav/state verifier) drives PLAN->TRACK->SUCCEEDED, blocked->REPLAN->RETRY
->resume->SUCCEEDED, unreachable->ABORTED, and the single-/cmd_vel-writer check.
The node reads every param via ~param with a code default the YAML overrides; the
live subset is also tunable at runtime. Remaining: the owner-in-the-loop RViz
live-motion gate (non-blocking acceptance).

## Boundary (do not break)
- Integration boundary is ROS-topic IPC. Do NOT link g1_costmap_core,
  g1_global_planner_core, or g1_local_planner_core.
- Core lib (g1_nav_core: fsm.cpp, progress_watchdog.cpp, retry.cpp) is ROS-free
  and std::thread-free. No ROS headers in include/g1_nav/core/ or src/core/.
- g1_nav_node is the SOLE writer of /cmd_vel. Do not add other /cmd_vel publishers.
- Freshness guards (do not remove): the map<-base_link last-good TF is reused only up to
  tf_timeout; past that have_pose=false and the mux is FORCED to zero -- never command
  motion on a frozen pose. local_state from /g1_local_planner/state is gated before the
  FSM sees it: substituted with NO_PATH while within goal_ack_grace of a new goal (the
  local planner is NOT disabled on a goal-to-goal preempt, so it briefly still reports
  the previous goal) or when older than local_state_ttl (planner stalled); this is the
  node's job, the FSM stays a pure function. When commanding nonzero /cmd_vel but
  LocoStatus.mode != MODE_WALK, warn (the robot is dropping the command).
- g1_nav only COORDINATES sub-node I/O: it forwards coordinated topic names as <arg>
  to each sub-launch (local: cmd_topic=/nav/cmd_vel_track, goal_topic=/nav/goal;
  global: goal_topic=/nav/goal), and the sub-launch applies them to its own node.
  NEVER use a <remap> child of an <include> -- roslaunch silently drops it (honors
  only <arg>/<env> there), so it would be a no-op and the local planner would become
  a second /cmd_vel writer. test_nav_launch.py guards this.
- auto_arm=true: on action accept g1_nav calls /g1/arm; on any terminal or cancel it
  calls /g1/halt. Gate on cfg.auto_arm (set false for sim/bench). Arming alone cannot
  move the robot: the locomotion walk-FSM whitelist is the hard gate, and the hardware
  e-stop is the true kill.
- No hardcoded values: all thresholds from NavConfig / WatchdogConfig
  (code defaults, YAML overrides, live knobs via dynamic_reconfigure).

## Layout
- include/g1_nav/core/nav_types.h  -- Pose2D, NavState, LocalState, MuxMode, NavOutcome,
                                      NavConfig, WatchdogConfig, Observations, NavDecision
                                      (NavState::RECOVER + NavOutcome::ABORTED_RECOVERY_EXHAUSTED
                                      are RESERVED: recovery removed, never emitted)
- include/g1_nav/core/fsm.h        -- NavFsm class declaration
- src/core/fsm.cpp                 -- NavFsm::step
- include/g1_nav/core/retry.h      -- RetryAction + decideRetryAction (RETRY decision)
- src/core/retry.cpp               -- decideRetryAction (TIME budget, not count; TIMEOUT dominates ATTEMPT)
- include/g1_nav/core/progress_watchdog.h -- ProgressWatchdog class declaration
- src/core/progress_watchdog.cpp   -- ProgressWatchdog (rolling-window stuck detector)
- src/nav_node.cpp                 -- g1_nav_node (the thin ROS orchestrator)
- config/nav.yaml                  -- flat params (structural + live defaults)
- cfg/NavTuning.cfg                -- dynamic_reconfigure live knobs
- launch/nav.launch                -- opt-in upstream args + single-writer/sole-goal topic args (forwarded to sub-launches)
- config/nav.rviz                  -- RViz view (costmap + paths + 2D Nav Goal + FSM state)
- test/test_fsm.cpp                -- FSM gtests (incl. STUCK/no_progress/timeout -> RETRY)
- test/test_progress_watchdog.cpp  -- ProgressWatchdog gtests (5 cases)
- test/test_nav_launch.py          -- python launch/config contract test
- test/nav_integration.test        -- rostest: g1_nav_node vs the mock harness
- test/nav_integration_test.py     -- the single mock node (upstreams + client + verifier)

## Node (src/nav_node.cpp)
Thin ROS orchestrator. AsyncSpinner(2) + one mutex mu_. 20 Hz timer order (exact):
(1) TF map<-base_link OFF the lock (last-good reuse on miss); (2) LOCK mu_:
watchdog_.update (in TRACK), build Observations (consume + clear the pending_* edges,
derive costmap_available = have_local_state && local_state!=NO_COSTMAP), fsm_.step,
execute core intents (reset watchdog), compute the mux Twist (RELAY_TRACK only if
track_cmd fresh within track_cmd_ttl else zero), snapshot for post-lock; (3) UNLOCK
then do ALL ROS I/O: publish /cmd_vel + /nav/state every tick, forward goal,
enable/disable/arm/halt service calls, action result + feedback.

Threading (hard): NEVER call a SimpleActionServer result method or a blocking service
.call() while holding mu_ (the AS-lock/mu_ deadlock + blocking-under-lock are the two
failure modes). Gather under mu_, release, then act. The core helpers
(fsm_/watchdog_) are touched ONLY under mu_. The last-good TF cache is
timer-thread-only (no extra lock).

Action lifecycle: goalCallback acceptNewGoal + validate (reject non-GOAL_POSE /
non-finite / untransformable pose with OUTCOME_REJECTED via setAborted, no active
flag); else set pending_new_goal_ + action_goal_active_. preemptCallback sets
pending_cancel_ only if !as_.isNewGoalAvailable() (a new-goal-driven preempt is
superseded by the new goal, not double-handled). RViz goalCb is ignored while
action_goal_active_. action_goal_active_ gates whether finish_action calls an AS
result (RViz goals never call one).

## Key decisions
- g1_nav is the sole /cmd_vel writer (local planner remapped to /nav/cmd_vel_track)
- FSM states: IDLE PLAN TRACK REPLAN RETRY SUCCEEDED ABORTED. NavState::RECOVER (=4)
  is a RESERVED, never-emitted wire constant (recovery removed); do NOT renumber.
- RETRY is the SOLE soft-failure handler: PLAN/REPLAN timeout AND TRACK STUCK /
  no_progress all route to RETRY -- hold + re-forward goal every retry_replan_interval
  up to retry_budget (~120s), resume to TRACK on a fresh path, abort
  ABORTED_BLOCKED_TIMEOUT on budget timeout. Ignores goal_timeout. There is NO blind
  recovery motion and NO costmap-clear: a genuine wedge holds + replans, then aborts.
- auto_arm: arm on accept, halt on terminal/cancel; toggle via config
- ROS-free core + thin node; /nav/state as std_msgs/Int8

## Build / test
```
catkin build g1_nav && source devel/setup.bash
catkin run_tests g1_nav --no-deps && catkin_test_results build/g1_nav
```
