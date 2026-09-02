# G1 Navigation FSM Orchestrator (g1_nav) 🧭

**Navigation orchestration layer** for the Unitree G1. It accepts a `navigate_to`
action goal (or an RViz **2D Nav Goal**), forwards it to the already-separate
planner stack (`g1_costmap`, `g1_global_planner`, `g1_local_planner`), runs a small
state machine that tracks the goal and recovers when stuck, gates `/cmd_vel` as the
SINGLE writer the locomotion bridge reads, and reports progress over the action.
This is a THIN COORDINATOR: it never re-implements planning, tracking, or costmap
logic, it only sequences the existing ROS-topic-IPC nodes. A ROS-free core
(`g1_nav_core`: `NavFsm` + `RecoverySequencer` + `ProgressWatchdog`, a pure function
of observations and time) sits behind a thin I/O node (`g1_nav_node`) that owns the
action server, TF, the 20 Hz mux+control timer, the service clients, and
`dynamic_reconfigure`. `g1_local_planner` produces the tracking velocity;
`g1_nav` is the FSM, recovery, gating, and goal management around it.

## 🧱 What you get

- **`navigate_to` action server** (`g1_msgs/NavigateToAction`): send a `GOAL_POSE`,
  get standard preempt/cancel and per-goal feedback (state, pose,
  distance-remaining, recovery attempt) plus a terminal result with an outcome code.
- **Single-writer `/cmd_vel` mux**: the node is the only publisher of `/cmd_vel`. It
  publishes every tick (zero when idle) so the locomotion watchdog always sees a
  fresh stream, sourcing the command from the FSM mux mode (relay the tracker, drive
  a recovery velocity, or zero).
- **A lean FSM** (`IDLE -> PLAN -> TRACK -> REPLAN | RECOVER | RETRY -> SUCCEEDED | ABORTED`)
  with stuck detection, a per-goal timeout, and goal tolerance owned by the local
  planner (no second tolerance check here).
- **A blocked-goal `RETRY` state**: when no valid path exists, the robot
  holds position and keeps re-planning for up to `retry_budget` (~2 min) instead of
  failing immediately, resuming the moment a path opens (config toggle `retry_enabled`).
- **A fixed recovery escalation** (CLEAR -> ROTATE -> BACKUP -> WAIT) with per-cycle
  caps and a total-time budget.
- **Optional auto-arm/halt** of `g1_locomotion` around a goal (config toggle).
- **Live tuning** of the timeouts, watchdog, and recovery knobs via
  `dynamic_reconfigure`; everything else is structural YAML.

## 🔌 IPC boundary (ROS topics + services + action)

| Direction | Name (param, default)                         | Type                        | Notes                                                                    |
|-----------|-----------------------------------------------|-----------------------------|--------------------------------------------------------------------------|
| action    | `action_name` `navigate_to`                   | `g1_msgs/NavigateToAction`  | `SimpleActionServer`; `GOAL_POSE` supported, semantic goal types REJECTED |
| sub       | `goal_topic` `/move_base_simple/goal`         | `geometry_msgs/PoseStamped` | RViz **2D Nav Goal**; ignored while an action goal is active             |
| sub       | `local_state_topic` `/g1_local_planner/state` | `std_msgs/Int8`             | latest `LocalState` from the local planner                               |
| sub       | `track_cmd_topic` `/nav/cmd_vel_track`        | `geometry_msgs/Twist`       | local planner tracking command (relayed in TRACK)                        |
| sub       | `loco_status_topic` `/g1/loco_status`         | `g1_msgs/LocoStatus`        | `halted` latch (informational)                                           |
| pub       | `cmd_vel_topic` `/cmd_vel`                     | `geometry_msgs/Twist`       | the SINGLE writer, every tick                                            |
| pub       | `goal_out_topic` `/nav/goal` (latched)        | `geometry_msgs/PoseStamped` | forwarded goal the planners consume                                      |
| pub       | `state_topic` `/nav/state` (latched)          | `std_msgs/Int8`             | `NavState` each tick                                                     |
| client    | `enable_service` `/g1_local_planner/enable`   | `std_srvs/SetBool`          | enable/disable the local planner                                         |
| client    | `clear_service` `/g1_costmap/clear_costmap`   | `std_srvs/Empty`            | dynamic-layer clear (the CLEAR recovery step)                           |
| client    | `arm_service` `/g1/arm`                        | `std_srvs/Trigger`          | only when `auto_arm`                                                     |
| client    | `halt_service` `/g1/halt`                      | `std_srvs/Trigger`          | only when `auto_arm`                                                     |

TF: `map_frame` (`map`) <- `robot_base_frame` (`base_link`), looked up off the lock
with last-good reuse on a miss. The integration boundary is ROS-topic / action IPC:
`g1_nav` does NOT link `g1_costmap_core`, `g1_global_planner_core`, or
`g1_local_planner_core`.

## 📦 First-time setup

Build the workspace (see `docs/INSTALL.md`). `g1_nav` has no non-ROS build
dependency; it needs the `navigate_to` action from `g1_msgs` and, at runtime, the
upstream nodes it coordinates (`g1_costmap`, `g1_global_planner`, `g1_local_planner`,
`g1_locomotion`). The python contract test needs `python3-nose` + `python3-yaml`.

## 🚀 Run

`g1_nav_node` is always launched by `nav.launch`; the upstream nodes are opt-in args
(default **off**), per the launch-composition convention, so the launch never
fabricates state it does not bring up.

```bash
# g1_nav only (the planner sub-nodes already running separately):
roslaunch g1_nav nav.launch

# Full OFFLINE nav stack with RViz (no robot, no locomotion):
roslaunch g1_nav nav.launch start_local:=true start_global:=true start_costmap:=true \
    start_slam:=true map_path:=/path/to/map rviz:=true

# Full LIVE stack (all sub-nodes + robot; e-stop in hand):
roslaunch g1_nav nav.launch start_local:=true start_global:=true start_costmap:=true \
    start_slam:=true start_frontend:=true start_lidar:=true start_state:=true \
    start_locomotion:=true map_path:=/path/to/map rviz:=true
```

`g1_nav` is the sole goal authority: the launch remaps the global and local planner
`/move_base_simple/goal` to `/nav/goal` (so only `g1_nav` forwards goals) and the
local planner `/cmd_vel` to `/nav/cmd_vel_track` (so only `g1_nav` writes `/cmd_vel`).
`g1_nav`'s own `goal_topic` stays `/move_base_simple/goal`, so the RViz **2D Nav
Goal** tool reaches it directly. Send a goal with the RViz tool, an action client,
or `rostopic pub /move_base_simple/goal ...`.

## 🧭 How it works

Each 20 Hz tick the node looks up the robot pose via TF, ticks the `ProgressWatchdog`
(while tracking) and the `RecoverySequencer` (while recovering), builds an
`Observations` struct, calls `NavFsm::step(obs, now)` under one mutex, executes the
returned `NavDecision` intents (forward the goal, enable/disable the local planner,
arm/halt, clear the costmap, finish the action), computes the mux `Twist`, and after
releasing the lock publishes `/cmd_vel` + `/nav/state` and performs all ROS I/O.

**The FSM** (`/nav/state` as `std_msgs/Int8`, also the action feedback `state`):

`IDLE -> PLAN -> TRACK -> (REPLAN | RECOVER | RETRY) -> SUCCEEDED | ABORTED -> IDLE`

- **IDLE**: no active goal; `/cmd_vel` zero.
- **PLAN**: goal forwarded; waiting to acquire a path + costmap and start tracking.
- **TRACK**: relaying the local planner's tracking command on `/cmd_vel`.
- **REPLAN**: re-nudged the goal; waiting for a fresh path after a debounced
  `NO_PATH`.
- **RECOVER**: running the recovery sequence (CLEAR -> ROTATE -> BACKUP -> WAIT) for a
  physical block (local `STUCK` / no-progress watchdog).
- **RETRY**: blocked goal / no valid path. Holds position (zero `/cmd_vel`) and
  re-forwards the goal every `retry_replan_interval` for up to `retry_budget`,
  resuming to TRACK the moment a fresh path appears, and aborting with
  `OUTCOME_ABORTED_BLOCKED_TIMEOUT` only after the budget elapses. This is the
  ~2 min RETRY budget; `goal_timeout` does not apply here.
- **SUCCEEDED / ABORTED**: terminal (one tick for `/nav/state` visibility), then
  IDLE.

A new goal preempts any state; a cancel returns to IDLE. Goal-reached authority is
the local planner's `GOAL_REACHED` (its xy/yaw tolerances are the single source of
truth); `g1_nav` does one action-level confirmation, not a parallel check.

**The single-writer `/cmd_vel` mux** sources the command by FSM mux mode:

- `RELAY_TRACK` (TRACK): relays the cached `/nav/cmd_vel_track`, but only if it is
  fresh (within `track_cmd_ttl`, default 0.5 s); a stale command relays as zero.
- `RECOVERY_VEL` (RECOVER): the `RecoverySequencer` body velocity (rotate / backup).
- `ZERO` (everything else): a zero `Twist`.

**Recovery** runs node-side while in RECOVER: a fixed escalation CLEAR (clear the
dynamic costmap layer) -> ROTATE (search for a clear heading) -> BACKUP (retreat a
short fixed distance) -> WAIT (hold for a dynamic obstacle to clear), repeated up to
`max_cycles` and bounded by `total_timeout`. After each behavior the FSM re-attempts
PLAN; a re-failed PLAN escalates to the next behavior. When the budget or cycle cap
is hit, the FSM falls into `RETRY` so a still-blocked goal
keeps re-planning before the single final give-up; with `retry_enabled=false` it
instead aborts immediately with `OUTCOME_ABORTED_RECOVERY_EXHAUSTED` (legacy).

**Blocked-goal `RETRY`** (`retry_enabled` default true): the
"no valid path" transitions (PLAN/REPLAN timeout) and recovery-exhausted funnel here
instead of aborting. The robot holds and re-forwards the goal every
`retry_replan_interval` (default 1 s) for up to `retry_budget` (default 120 s),
returns to TRACK on the first fresh path, and only then -- if the budget elapses --
aborts with `OUTCOME_ABORTED_BLOCKED_TIMEOUT`. The decision is a pure, unit-tested
function (`decideRetryAction`); the budget is by time, not attempt count. So a closed
door, a crowd, or a SLAM settle no longer fails the goal in ~30 s.

**auto_arm**: when `auto_arm` is true the FSM emits `arm` on goal accept and
`halt` on any terminal or cancel, and the node forwards these to `/g1/arm` and
`/g1/halt`. **Arming alone cannot move the robot**: the locomotion walk-FSM whitelist
is the hard gate (`arm` only clears the halt latch; streaming still needs a
whitelisted walk FSM + a non-zero `/cmd_vel`), and the hardware e-stop is the true
kill. Set `auto_arm` false for sim / bench so no arm/halt is emitted.

## ⚙️ Tuning

The FSM timeouts (`plan_timeout`, `replan_timeout`, `goal_timeout`, `no_path_grace`,
`auto_arm`), the blocked-goal RETRY (`retry_enabled`, `retry_budget`,
`retry_replan_interval`), the watchdog (`stuck_window`, `stuck_dist`), and the
recovery knobs (`total_timeout`, `max_cycles`, the rotate / backup / wait / clear
params and their enables) are live-tunable via `dynamic_reconfigure`:

```bash
rosrun rqt_reconfigure rqt_reconfigure   # adjust /g1_nav at runtime
```

Structural params (topics, services, frames, `rate`, `transform_tolerance`,
`feedback_rate`, `track_cmd_ttl`, `service_connect_timeout`) are static YAML
(`config/nav.yaml`) and take effect at startup. No value is hardcoded in the node:
every field has a code default the YAML overrides and the live subset overrides
again at runtime.

## 🧪 Build & test

```bash
# from catkin_ws/:
catkin build g1_nav && source devel/setup.bash
catkin run_tests g1_nav --no-deps && catkin_test_results build/g1_nav
```

The ROS-free core gtests cover the FSM transitions (including the `RETRY`
blocked-goal RETRY), the `decideRetryAction` decision function, recovery escalation,
and the progress watchdog. A python launch/config contract test guards the launch
remaps and the YAML/param contract. An end-to-end `rostest`
(`test/nav_integration.test`) brings up `g1_nav_node` against a single
event-driven mock node (mock upstreams + action client + verifier) and exercises the
FSM end-to-end: a clean run reaches `SUCCEEDED`, a blocked run drives
`REPLAN -> RETRY` and resumes to `SUCCEEDED` once the path clears, an unreachable
goal runs `RECOVER -> RETRY` and aborts with `ABORTED_BLOCKED_TIMEOUT`, and
`/cmd_vel` has exactly one publisher:

```bash
rostest g1_nav nav_integration.test
```

The remaining acceptance item is the owner-in-the-loop RViz live-motion gate
(e-stop in hand): an action goal auto-arms, plans, tracks, and stops aligned; a moved
obstacle triggers REPLAN/RECOVER and the robot re-reaches the goal.
