# G1 Global Planner (g1_global_planner) 🧭

Plain **A\* global path planner** for the Unitree G1. It searches the fused costmap
(`/nav/costmap`) for a **collision-free, inflation-aware** path from the robot to an
RViz goal, smooths it with line-of-sight shortcutting, and publishes it as
`/nav/global_path` for the local MPC planner. The planner is 8-connected A* with the
octile heuristic and no diagonal corner-cutting; the costmap's inflation gradient is
folded into the step cost so paths keep clearance from obstacles instead of grazing
them. Reimplemented in our own C++ (no `move_base` / `nav_core` / `global_planner`
plugin stack): a ROS-free core (`g1_global_planner_core`: grid view + cost model +
A* + smoother + a `GlobalPlanner` facade) behind a thin I/O node, with the costmap
read as the standard `nav_msgs/OccupancyGrid` message (this package does NOT link
`g1_costmap`).

## 🧭 What you get
- `/nav/global_path` (`nav_msgs/Path`, `map` frame) - the planned path: a
  line-of-sight-smoothed polyline of poses with segment-tangent yaw (the last pose
  carries the goal yaw). An empty `Path` is published on a plan failure (no path,
  or a blocked start/goal) with a throttled warning, never a crash
- Goal in via `/move_base_simple/goal` (`geometry_msgs/PoseStamped`, the RViz
  **2D Nav Goal** tool); start from the `map`->`base_link` TF
- **Inflation-aware A\***: lethal/inscribed cells (OccupancyGrid 100) and unknown
  (-1) are blocked; the 1..99 inflation gradient adds step cost so the path prefers
  open corridors. Unknown is blocked by default (conservative for a humanoid),
  tunable via `allow_unknown`
- **Replan** on a new goal, or when a costmap update blocks the current path
  (path-deviation replanning is handled by the local MPC re-tracking in `g1_local_planner`, not
  here). The planning tick runs at `update_rate` 1 Hz (slow global failsafe + block-check;
  the fast 20 Hz tracking is `g1_local_planner`), so a
  new goal or a costmap-block is acted on within ~1 s
- A ROS-free, unit-testable core behind a thin I/O node; the integration boundary
  is ROS topics. Live tuning of the cost/heuristic/smoothing/snap knobs via
  `dynamic_reconfigure`

This node plans the **global path** only; the MPC tracker that follows it
(`g1_local_planner`) and the FSM (`g1_nav`) handle motion execution.

## 📦 First-time setup
- Build the workspace (see `docs/INSTALL.md`). No special third-party deps: the
  planner uses standard ROS message types only (no PCL, no Eigen).

## 🚀 Run
```bash
# g1_costmap (or a bag) already publishing /nav/costmap + the map->base_link TF:
roslaunch g1_global_planner global_planner.launch
# + RViz (costmap + path; use the "2D Nav Goal" tool to send a goal):
roslaunch g1_global_planner global_planner.launch rviz:=true
# bring up the costmap too (SLAM up elsewhere):
roslaunch g1_global_planner global_planner.launch start_costmap:=true rviz:=true
# costmap + SLAM backend (sensors/state up elsewhere):
roslaunch g1_global_planner global_planner.launch start_costmap:=true start_slam:=true rviz:=true
# full live stack (planner + costmap + backend + frontend + sensor + robot state):
roslaunch g1_global_planner global_planner.launch start_costmap:=true start_slam:=true start_frontend:=true start_lidar:=true start_state:=true rviz:=true
```
This node only **consumes** `/nav/costmap`, the goal, and TF. Per the
launch-composition convention, opt-in args (default off) bring up the upstream:
`start_costmap` launches `g1_costmap`, and its `start_slam` / `start_frontend` /
`start_lidar` / `start_state` / `map_path` toggles are forwarded through.

## 🧭 How it works
- **Costmap as a grid view.** The incoming `/nav/costmap` is copied into a ROS-free
  `PlannerGrid` (resolution, origin, width/height, the `int8` OccupancyGrid cells).
  The `CostModel` reads each cell: `100` (lethal/inscribed) or `-1` (unknown, unless
  `allow_unknown`) is blocked; `0..99` is traversable with step cost
  `1 + inflation_cost_weight * v/100`, so the inflation halo steers the path.
- **A\* search.** 8-connected, edge cost = move distance (1 or sqrt(2)) times the
  destination cell's cost factor, octile-distance heuristic (admissible at
  `heuristic_weight` 1.0; > 1 is faster weighted/greedy but suboptimal). A diagonal
  step is forbidden when either shared orthogonal neighbour is blocked (no
  corner-cutting), so a path never clips an obstacle corner. `max_expansions` caps
  the search so an unreachable goal cannot stall the node.
- **Line-of-sight smoothing.** The 8-connected cell path is string-pulled: greedily
  connect the farthest waypoint whose straight segment is collision-free (an exact
  integer supercover grid traversal checks every cell the segment crosses), turning
  the staircase into a clean polyline. The same line-of-sight test re-validates a
  stored path against a fresh costmap for the replan trigger.
- **Goal robustness.** If the start or goal cell is blocked (an RViz goal often
  lands in an inflation halo), it snaps to the nearest traversable cell within
  `goal_snap_radius`; outside that it fails cleanly (`BAD_START` / `BAD_GOAL`).
- **Plan loop.** The costmap and goal callbacks only cache (cheap); a fixed-rate
  timer does the start TF lookup, decides whether to plan (new goal, new costmap
  with no valid path, or the stored path now blocked), runs the planner, and
  publishes. One mutex guards the caches and the planner; planning runs under the
  lock (the costmap node pattern) and only the publish happens after release. A TF
  miss reuses the last good transform (throttled).

## ⚙️ Tuning
All knobs/topics/frames live in `config/global_planner.yaml` (loaded into the node's
private namespace; every field has a code default the YAML overrides). The
live-tunable subset is exposed via `dynamic_reconfigure` (`cfg/PlannerTuning.cfg`):
`lethal_threshold`, `allow_unknown`, `inflation_cost_weight`, `heuristic_weight`,
`smooth_enable`, `goal_snap_radius`, `replan_on_block`. Structural params
(`map_frame`, `robot_base_frame`, topics, `update_rate`, `transform_tolerance`,
`allow_corner_cutting`, `max_expansions`) are static YAML.

> **Cost vs. clearance.** Raise `inflation_cost_weight` to push paths further off
> obstacles (longer, safer); lower it for shorter, tighter paths. Raise
> `heuristic_weight` above 1.0 to plan faster on large maps at the cost of
> optimality. Set `allow_unknown: true` only when you accept routing through
> unmapped space. Set `max_expansions` > 0 to bound worst-case latency when goals
> may be unreachable.

## 🧪 Build & test
```bash
# from catkin_ws/:
catkin build g1_global_planner && source devel/setup.bash
catkin run_tests g1_global_planner --no-deps && catkin_test_results build/g1_global_planner
```
C++ unit tests: the grid world<->cell math, the cost model (lethal/unknown/halo
blocking, step-cost monotonicity), A* (optimal paths, wall detour, no-path,
bad-start/goal, unknown blocking both ways, the expansion cap, two-sided
no-corner-cut), the line-of-sight supercover smoother (staircase collapse, obstacle
detours, asymmetric-segment crossing cases), and the `GlobalPlanner` facade
(end-to-end collision-free paths, goal snap in/out of radius, path-validity
replan trigger). A python check guards the launch/config contract. The integration
gate is an offline replay (the real `/nav/costmap` built from a saved SLAM map) +
RViz goals, with a moved obstacle to exercise the replan.

This node is the global path planning layer: A* over the inflated costmap, smoothed
path out, consumed downstream by `g1_local_planner` (MPC tracking) and `g1_nav` (FSM).
