# g1_global_planner - agent guide

Plain A* 2D global path planner. In: /nav/costmap (nav_msgs/OccupancyGrid, inflated,
map frame, from g1_costmap), /move_base_simple/goal (geometry_msgs/PoseStamped, RViz
2D Nav Goal), TF map<-base_link (start). Out: /nav/global_path (nav_msgs/Path, map
frame) for the g1_local_planner MPC local planner. 8-connected A* + octile heuristic + no
corner-cut, inflation-aware cost, line-of-sight (supercover) smoothing. Own C++
reimplementation; no move_base/nav_core/global_planner plugin stack. JPS engine
(optional) deferred.

## Boundary (do not break)
- Integration boundary is ROS-topic IPC: read /nav/costmap as a standard message;
  do NOT link g1_costmap_core. Other packages consume /nav/global_path. This node
  owns NO TF.
- This node plans the GLOBAL path only. No motion (motion is handled by g1_local_planner
  MPC + g1_nav FSM). No path-deviation replanning here (needs a moving robot).
- plan(start,goal) SERVICE is deferred until g1_nav needs it; goal is a topic only.

## Layout
Headers under include/g1_global_planner/{core,planner}/, sources under
src/{core,planner}/ (parallel); the node stays at src/ root. All core code is
ROS-free, std::thread-free, unit-testable, built as lib g1_global_planner_core.
- core/ (data model + pure rules): plan_types.h (Cell, Pose2D, PlanStatus,
  PlannerConfig with all knobs+defaults, PlanResult), planner_grid.{h,cpp}
  (PlannerGrid: res/origin/w/h + int8 OccupancyGrid cells, world<->cell),
  cost_model.{h,cpp} (CostModel: blocked(v), stepCost(v)).
- planner/: astar.{h,cpp} (8-connected A*), path_smoother.{h,cpp} (lineOfSight +
  smoothPath), global_planner.{h,cpp} (GlobalPlanner facade: plan + pathStillValid +
  snap).
- Node (ROS I/O only): src/planner_node.cpp -> g1_global_planner_node. The launch
  (launch/global_planner.launch) exposes a goal_topic arg (standalone default
  /move_base_simple/goal) and applies it as a <param> override on the node, so a
  coordinator (g1_nav) routes this node's goal by forwarding the arg -- this node owns
  the wiring, the coordinator never reaches in with a remap.

## Invariants (do not break)
- **Cost model on the OccupancyGrid value v.** The costmap now publishes 100=lethal,
  99=inscribed, 1..98=inflation. v>=lethal_threshold (default 99) or (v<0 unknown and
  !allow_unknown) -> blocked, so TRAVEL blocks both inscribed(99) and lethal(100).
  Traversable cell step cost = move_dist(1 or sqrt2) * (1 + inflation_cost_weight*v/100).
  Unknown is BLOCKED by default (allow_unknown false): the global grid rim is unknown
  and a humanoid must not route through unmapped space.
- **Goal-local relax (Track B).** Within goal_relax_radius of the goal, relaxAroundGoal
  downgrades inscribed(99) cells to the top traversable value (lethal_threshold-1), so
  only true-lethal(100) blocks there and a goal near an obstacle is reachable. Applied
  in BOTH plan() (around the goal) and pathStillValid() (around the path endpoint, so a
  legitimate near-goal path is not perpetually re-invalidated). The planner stays a
  POINT planner with no footprint dependency; the g1_local_planner footprint filter is
  the safety guarantee in the relaxed band. Disable via goal_relax_enable.
- **A* = 8-connected, octile heuristic, NO corner-cut.** A diagonal step needs BOTH
  shared orthogonal neighbours unblocked (else it could clip an obstacle corner).
  Heuristic admissible at heuristic_weight 1.0 (stepCost >= 1); >1 is weighted/greedy.
  max_expansions caps the search (0 = unlimited) so an unreachable goal cannot stall
  the timer. Deterministic tie-break (min-f, then max-g, then min-index).
- **lineOfSight is an exact integer supercover DDA (Amanatides-Woo).** It must visit
  EVERY cell the segment crosses and is conservative at exact corner crossings (checks
  both shared cells). The smoother and pathStillValid TRUST it to keep a path off
  lethal/unknown cells - a thin/Bresenham variant would let a smoothed segment clip an
  obstacle (a safety defect; verified by an oracle over 558k cell pairs). Do NOT
  replace it with a single-error-term line.
- **Smoothing = line-of-sight string-pull only; spline deferred to the local MPC layer.** Greedily
  connect the farthest waypoint with a collision-free straight segment. smooth_enable
  off -> raw 8-connected polyline. The MPC consumer (g1_local_planner) defines any further
  smoothness; do not add a spline here speculatively.
- **Event-driven new goal + transient-failure hysteresis (do not remove).** goalCb
  forces an immediate replan (timer setPeriod(0) -- the timer's callbacks are serialized
  so the last-good TF cache stays timer-thread-only) instead of waiting up to 1 s, so a
  new goal does not leave the OLD latched path serving the local planner. The new_* edge
  flags are NOT cleared until a pose is acquired (else a goal arriving during a TF
  dropout is lost). A plan FAILURE on an established (non-new) goal is tolerated up to
  replan_fail_tolerance consecutive ticks before wiping the latched path to empty -- the
  local planner is the fast reactor for transient obstacles, so one bad tick must not
  drop a still-good path. A frozen start TF (older than tf_timeout) skips planning.
- **Replan triggers (no motion): new goal, OR new costmap with no valid path, OR
  replan_on_block && stored path now invalid (pathStillValid false).** Path-deviation
  replanning stays out of this node (the robot's deviation is absorbed by the
  local MPC re-tracking in g1_local_planner, not a global replan). pathStillValid is the cheap gate before
  a full replan. **Rate split:** update_rate is 1.0 Hz (dropped from 5.0 Hz); this is now
  the SLOW global failsafe + block-check tick (the fast 20 Hz tracking and immediate
  obstacle reaction is g1_local_planner). A NEW GOAL already triggers an immediate
  event-driven replan (goalCb setPeriod(0); see the event-driven new-goal invariant
  above), so its latency is NOT bounded by this rate. A costmap-block / new-costmap is
  still acted on at the next failsafe tick (within ~1 s); making the costmap/block
  callback also fire an immediate replan is a small future option if lower global
  block-response latency is wanted.
- **Goal snap.** A blocked start/goal snaps to the Euclidean-nearest traversable cell
  within goal_snap_radius (0 = off, then a blocked endpoint fails BAD_START/BAD_GOAL).
  Keeps RViz goals that land in an inflation halo usable; this is NOT the FSM goal
  tolerance (that is handled by g1_nav).
- **Params in the node PRIVATE namespace**, FLAT keys from config/global_planner.yaml
  so dynamic_reconfigure reads the live subset directly. Every param has a code
  default the YAML overrides. Live knobs via cfg/PlannerTuning.cfg ->
  g1_global_planner::PlannerTuningConfig (named to avoid clashing with the core
  g1_global_planner::PlannerConfig struct). max_expansions + allow_corner_cutting are
  structural YAML (not live).
- **Threading.** Core is std::thread-free. The node runs AsyncSpinner(2) with one
  mutex guarding the cached grid/goal/flags + the planner. The timer snapshots caches
  under the lock, does the start TF lookup OFF the lock, then runs plan()+pathStillValid
  UNDER the lock (so reconfigure's setConfig cannot race the planner) and publishes the
  Path AFTER releasing. TF miss reuses the last good transform (throttled). The
  last-good TF cache is timer-thread-only.

## Build / test
```bash
# from catkin_ws/:
catkin build g1_global_planner && source devel/setup.bash
catkin run_tests g1_global_planner --no-deps && catkin_test_results build/g1_global_planner
```
C++ gtests (grid, cost model, astar, smoother incl. supercover oracle cases, facade)
+ a python launch/config contract check. The integration gate is an offline replay
(the real /nav/costmap built from a saved SLAM map) over many start/goal pairs
(collision-free + endpoint + progress) + RViz goals with a moved obstacle for the
replan. Node binary g1_global_planner_node; node name g1_global_planner.

This node is the global path planning layer: A* over the inflated costmap, smoothed
path out, consumed downstream by g1_local_planner (MPC tracking) and g1_nav (FSM).
