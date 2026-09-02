# G1 Local Planner (g1_local_planner) 🚗

**Linear MPC local trajectory tracker** for the Unitree G1. It follows the global
path (`/nav/global_path`) while avoiding obstacles in a rolling robot-centred costmap
window (`/nav/local_costmap`), looks up the current pose from TF, and emits a
body-frame velocity command (`/cmd_vel`) that drives `g1_locomotion`. The controller
is a Quadratic Program assembled every 20 Hz control cycle and solved with OSQP +
osqp-eigen. The omnidirectional kinematics are linearised about the current heading
(an LTI model), obstacles enter as a soft linearised costmap-gradient penalty sampled
over the robot's footprint (never a hard keep-out in the QP, so it stays feasible),
and the robot faces its direction of travel, strafing only to avoid or fine-align. The
robot is modelled as an explicit collision body (a lateral array of body-frame
circles), and a post-MPC **footprint safety filter** guarantees the applied command
never drives that footprint over a lethal cell (it scales the speed down and brakes to
a stop as the swept footprint nears an obstacle). Reimplemented in our own C++ (the prior
holonomic DWA stack is the behavioural reference only): a ROS-free core
(`g1_local_planner_core`: kinematic model + arc-length reference + costmap view +
MPC/QP assemble-and-solve + a `LocalPlanner` facade) behind a thin I/O node, with all
inputs read as standard `nav_msgs` / `geometry_msgs` messages (this package does NOT
link `g1_costmap_core` or `g1_global_planner_core`). This is the first phase that
produces robot MOTION; the navigation FSM, recovery, and `navigate_to` action are
`g1_nav`.

## 🔌 IPC boundary (ROS topics)

| Direction | Topic | Type |
|-----------|-------|------|
| In | `/nav/global_path` | `nav_msgs/Path` (map frame, from `g1_global_planner`) |
| In | `/nav/local_costmap` | `nav_msgs/OccupancyGrid` (map frame, rolling 6x6 m window, from `g1_costmap`) |
| In | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` (RViz **2D Nav Goal**, for goal yaw + reached) |
| In | TF `map -> base_link` | current robot pose |
| Out | `/cmd_vel` | `geometry_msgs/Twist` (`linear.x`=vx, `linear.y`=vy, `angular.z`=wz) |
| Out | `/nav/local_plan` | `nav_msgs/Path` (predicted MPC trajectory, map frame, for RViz) |
| Out | `~state` | `std_msgs/Int8` (`ControllerState` code) |

The integration boundary is ROS-topic IPC: the planner reads the standard messages
above and emits the standard `/cmd_vel`. It does **NOT** link `g1_costmap_core` or
`g1_global_planner_core`.

## 📦 First-time setup

Build the workspace (see `docs/INSTALL.md`). The only non-ROS build dependency is
OSQP + osqp-eigen (installed by `scripts/install_system_deps.sh`); Eigen comes from
`ros-noetic-desktop-full`. Runtime upstreams: `g1_costmap` (publishes
`/nav/local_costmap`) and `g1_global_planner` (publishes `/nav/global_path`).

## 🚀 Run

```bash
# Minimal: upstreams already publishing (costmap + global path + the map->base_link TF):
roslaunch g1_local_planner local_planner.launch
# + RViz (local costmap + global path + predicted local plan + the 2D Nav Goal tool):
roslaunch g1_local_planner local_planner.launch rviz:=true
# bring up the global planner too (costmap still external):
roslaunch g1_local_planner local_planner.launch start_global:=true
# full live stack from this launch (planner + global + costmap + SLAM backend):
roslaunch g1_local_planner local_planner.launch \
    start_costmap:=true start_global:=true start_slam:=true map_path:=/path/to/map rviz:=true
```

Per the launch-composition convention, opt-in args (default **off**) bring up the
upstream: `start_global` launches `g1_global_planner`, `start_costmap` launches
`g1_costmap`, and the `start_slam` / `start_frontend` / `start_lidar` / `start_state`
/ `map_path` toggles are forwarded through. With everything off the node only consumes
the topics + TF.

## 🛡️ Enable gate (default OFF, first-motion safety)

The node starts with `/cmd_vel` zeroed (`enable_on_start: false` in the YAML).
While disabled it still solves the MPC and publishes `/nav/local_plan` + `~state`, so
the operator can watch the predicted trajectory in RViz before arming. Arm it with the
`~enable` service (`std_srvs/SetBool`) once the predicted path looks right and the
e-stop is in hand:

```bash
rosservice call /g1_local_planner/enable "data: true"    # arm: applied u_0 -> /cmd_vel
rosservice call /g1_local_planner/enable "data: false"   # disarm: zero /cmd_vel
```

### ControllerState codes (`~state`, `std_msgs/Int8`)

| Code | Name | Meaning |
|------|------|---------|
| 0 | IDLE | No goal received; publishing zero `/cmd_vel` |
| 1 | TRACKING | Actively following the global path |
| 2 | GOAL_REACHED | Within xy and yaw goal tolerances |
| 3 | NO_PATH | Global path is empty or not yet received |
| 4 | NO_GOAL | No goal received yet |
| 5 | NO_COSTMAP | Local costmap not yet received |
| 6 | STUCK | MPC solve failed, or the footprint safety filter braked to a stop (footprint blocked); zero cmd issued |

## 🧭 How it works

- **Arc-length rolling reference.** The robot is projected onto `/nav/global_path`;
  `N` reference states are placed ahead along the path by arc length `v_target * dt` per
  step. `yaw_ref` is the path tangent so the robot faces its travel direction. Within
  `goal_align_radius` the heading reference switches to the goal yaw and the remaining
  arc length shrinks to zero, decelerating into the goal.
- **Reference speed regulation.** `v_target` is recomputed each cycle as the minimum of
  a curvature cap `sqrt(a_lat_max / kappa)` (slow in turns; `kappa` is the max
  `|dtheta/ds|` over a `curv_lookahead` window), an approach decel-to-stop ramp
  `sqrt(2*a_decel*remaining)` (smooth stop into the goal), and a heading-error factor
  that scales speed down between `heading_slow_start` and `heading_slow_full`
  (turn-in-place / reverse entry, keyed on the path tangent). The cruise terms are
  floored at `v_min_move`; the approach ramp may drive `v_target` to zero at the goal.
  The MPC then tracks a reference that already slows, so the whole horizon plans the
  deceleration.
- **LTI kinematics about the current heading.** The body-to-map rotation is fixed at
  `R(theta0)` over the whole 2 s horizon, so body velocity maps linearly to map
  displacement and the QP stays convex. Curve error is absorbed by re-solving at 20 Hz.
- **Footprint collision body.** The robot is a lateral array of equal-radius
  body-frame circles (default: radius 0.15 m, 5 circles spanning a 0.5 m width x
  0.3 m depth body), rotated by yaw so the body's extent matters at the correct
  heading. Inscribed radius 0.15 m, circumscribed 0.25 m; the geometry is config-driven
  (`footprint_*` in the YAML).
- **Soft obstacle penalty (footprint-aware).** The local costmap gradient is sampled
  over the rotated footprint circle centres at a nominal trajectory and averaged, then
  enters the QP as a linear push down-gradient, re-linearised each cycle
  (real-time-iteration SQP). There is no hard keep-out in the QP, so it stays feasible;
  the smooth inflation gradient replaces the reference stack's binary obstacle cliff.
  The global planner is the primary avoider; the local MPC handles tracking margin and
  dynamic obstacles.
- **Footprint safety filter (the hard guarantee).** After the QP solves, the filter
  checks the applied command's swept footprint against the costmap (a cell
  value `>= 100`, i.e. lethal or inscribed, or an unknown/off-grid cell, is a
  collision). If the footprint would touch such a cell it scales the speed down to the
  largest value that can still brake before contact (`v_safe = sqrt(2*acc_lim_x*d)`),
  and brakes to a stop at the boundary, reporting `STUCK`. The guarantee that the
  footprint never overlaps a lethal cell comes from this filter, not the QP. Live
  knobs: `safety_filter_enabled`, `safety_brake_margin`.
- **Constraints.** Box limits on `(vx, vy, wz)` and slew limits
  `|u_k - u_{k-1}| <= acc * dt`, with `u_{-1}` = the last applied command (open-loop
  self-referential). One MPC controller handles tracking and terminal align; there is
  no separate terminal P-controller.
- **Control loop.** Subs cache cheaply; a 20 Hz timer does the TF lookup off-lock
  (last-good reuse on miss), then `compute()` under one mutex, applies `u_0` to
  `/cmd_vel` (if enabled), and publishes the predicted `s_1..s_N` to
  `/nav/local_plan`. The 20 Hz loop is decoupled from (and faster than) the costmap
  publish rate.

## ⚙️ Tuning

All knobs/topics/frames live in `config/local_planner.yaml` (loaded into the node's
private namespace; every field has a code default the YAML overrides). The
live-tunable subset is exposed via `dynamic_reconfigure` (`cfg/LocalPlannerTuning.cfg`):

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Live knobs: the cost weights (`w_pos`, `w_yaw`, `w_obstacle`, `w_effort`, `w_lateral`,
`w_rate`, the terminal weights), `v_ref`, the velocity/accel limits, the goal
tolerances, `goal_align_radius`, the speed-regulation knobs (`a_lat_max`, `a_decel`,
`curv_lookahead`, `heading_slow_start`, `heading_slow_full`, `heading_slow_floor`,
`v_min_move`), and the safety filter (`safety_filter_enabled`,
`safety_brake_margin`). The deployed `acc_lim_x` is `0.5` (config + dynamic_reconfigure),
matched to `g1_locomotion` `ax_max` so the MPC plans braking the robot can execute (the
code default stays `1.5` for the MPC slew-mechanics test fixtures). Structural params (`map_frame`, `robot_base_frame`, topics,
horizon `N`=20, `dt`=0.1 s, `update_rate`, the window size, `enable_on_start`, the
`footprint_*` geometry, `control_dt`, the static `safety_*` fields) are static YAML.

> **Behaviour vs. caution.** Raise `w_obstacle` to push the predicted path further off
> obstacles; raise `w_lateral` to discourage strafing (more forward-facing walking);
> raise `w_pos` / `w_yaw` for tighter path/heading tracking. Lower `v_ref` for slow,
> safe first-motion tests. First-motion safety comes from the enable gate + the
> locomotion clamp + e-stop, not from cutting the limits.

## 🧪 Build & test

```bash
# from catkin_ws/:
catkin build g1_local_planner && source devel/setup.bash
catkin run_tests g1_local_planner --no-deps && catkin_test_results build/g1_local_planner
```

C++ unit tests cover the costmap view (lookup, saturation, gradient sign, bounds),
the kinematic model (body->map displacement, yaw integration, rollout), the arc-length
reference (spacing, curved tangents, near-goal deceleration + yaw switch), the MPC
(straight-reference tracking, box/slew limits never exceeded, an obstacle gradient
pushes the path off a lethal cell, a dense obstacle still returns a soft solution and
never throws), and the `LocalPlanner` facade (end-to-end corridor reach + collision
free, no-path/goal-reached zero commands). A python check guards the launch/config
contract.

The integration gate has two parts:
- **Offline closed-loop replay** (a git-ignored harness): it builds the real inflated
  costmap from a saved SLAM map, plans global paths over several free-corridor
  start/goal pairs, then drives the `LocalPlanner` facade closed-loop by integrating
  the TRUE nonlinear plant at `dt`, and asserts every rollout reaches the goal within
  tolerance, stays collision-free against the master costmap, and never exceeds the
  velocity/accel limits; it also runs a synthetic corridor with two inflated obstacle
  blocks so the obstacle term is always exercised.
- **RViz motion gate** (owner in the loop, e-stop in hand): a 2D Nav Goal yields a
  tracked `/nav/local_plan`; arm via `~enable`; the robot walks the path at tiny
  `v_ref`, strafes/yaws to avoid a moved obstacle, and stops aligned at the goal.

