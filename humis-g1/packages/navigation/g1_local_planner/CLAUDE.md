# g1_local_planner - agent guide

Linear MPC local tracker. In: /nav/global_path (nav_msgs/Path), /nav/local_costmap
(nav_msgs/OccupancyGrid, rolling 6x6 m window from g1_costmap), /move_base_simple/goal
(geometry_msgs/PoseStamped), TF map->base_link. Out: /cmd_vel (geometry_msgs/Twist:
linear.x=vx, linear.y=vy, angular.z=wz), /nav/local_plan (nav_msgs/Path), ~state
(std_msgs/Int8). Solved via OSQP + osqp-eigen (QP per 20 Hz cycle).

## Boundary (do not break)
- Integration boundary is ROS-topic IPC. Do NOT link g1_costmap_core or
  g1_global_planner_core. Reads standard nav_msgs messages only.
- No FSM, no recovery, no action server (those are g1_nav).
- /cmd_vel is enable-gated (std_srvs/SetBool ~enable, default OFF). When disabled
  the node still solves and publishes /nav/local_plan but emits zero cmd_vel.
- TF freshness (do not remove): the map<-base_link last-good is reused only up to
  tf_timeout; past that the tick does NOT run the MPC -- it emits STUCK + zero cmd, so
  it never tracks blind or latches a false GOAL_REACHED on a frozen pose.

## Layout
Headers under include/g1_local_planner/{core,planner}/, sources under
src/{core,planner}/ (parallel layout). All core code is ROS-free, std::thread-free,
unit-testable, built as lib g1_local_planner_core. osqp-eigen lives in the core.
- core/: plan_types.h (Pose2D, Twist2D, ControllerState, MpcConfig+defaults incl.
  footprint/safety/control_dt, MpcResult), costmap_view.{h,cpp} (ROS-free grid
  snapshot, world<->cell, bilinear cost(), finite-diff gradient()).
- core/robot_footprint.{h,cpp} -- the robot collision body: a lateral array of equal
  body-frame circles (FootprintCircle/FootprintConfig + SafetyConfig live here so
  plan_types.h can embed both without an include cycle). build() derives evenly-spaced
  centers + inscribed/circumscribed radii; worldSamples(x,y,yaw) rotates centers to
  world (orientation-aware). Default: r=0.15, 5 circles, 0.5 m wide x 0.3 m deep,
  circumscribed 0.25 (<= costmap robot_radius 0.3).
- core/footprint_safety.{h,cpp} -- post-MPC safety filter (the HARD guarantee).
  poseInCollision checks the full footprint DISKS (each circle's area, dilated by
  footprint_collision_margin, default 0) against true-lethal (lethal_value 100) /
  unknown(-1 when treat_unknown) / off-grid. Since the costmap now publishes
  inscribed(99) distinct from lethal(100), this checks the real body vs real
  obstacles only (honest ~circle_radius standoff; resolution floor ~half a cell).
  clearDistance (swept arc length to first collision over the predicted trajectory),
  filter (pass when clear, scale speed to v_safe=sqrt(2*acc_lim_x*d) near obstacles,
  brake to STUCK at the boundary). The guarantee lives here, not in the QP.
- core/kinematic_model.{h,cpp} -- LTI model about theta0: controlMatrix(theta0,dt)
  returns B so s_{k+1}=s_k+B*u; stepLinear (QP prediction, fixed theta0); stepTrue
  (nonlinear true plant, for offline sim only); rolloutLinear (returns s_1..s_N).
- planner/path_reference.{h,cpp} -- arc-length rolling reference: projectOntoPath /
  pointAtArcLength / generateReference; goal-align heading switch within
  goal_align_radius + reference end clamp so the MPC decelerates into the goal.
  referenceSpeed() computes the per-cycle target speed = min(v_ref, curvature cap
  sqrt(a_lat_max/kappa), heading factor f(|tangent-yaw|)) floored at v_min_move, then
  capped by the approach ramp sqrt(2*a_decel*remaining); generateReference spaces
  points by this v_target (not the flat v_ref).
- planner/mpc.{h,cpp} -- linear-MPC core: assembles the sparse QP (decision vector
  z=[s_1..s_N,u_0..u_{N-1}], dynamics as equalities, box+slew on u, tracking +
  linearised-obstacle + effort + strafe + rate costs) and solves it with osqp-eigen
  each cycle. Class Mpc::solve(current,theta0,refs,last_cmd,costmap) returns u_0 in
  .applied, s_1..s_N in .predicted, the full sequence in .controls, plus .solved/.cost.
  Reference yaws are unwrapped about theta0 so the tracking quadratic rotates the
  short way across +-pi. Obstacle term is linearised about a nominal: the reference
  points by default, or (warm start) last cycle's predicted path when solve()
  is passed obstacle_nominal of size == horizon -- re-linearising about the previous
  predicted trajectory (real-time-iteration SQP) so avoidance is self-reinforcing
  across cycles. Head-on caveat: a perfectly symmetric block on the reference line
  still has g_y ~ 0 on the first cycle (global planner is the primary avoider).
  last_cmd is clamped into the velocity box at solve entry so a stale out-of-box
  command can never make a k=0 slew row empty. On any solver failure returns
  solved=false with a zero command + zero rollout and never throws (soft-penalty feasibility).
- planner/local_planner.{h,cpp} -- LocalPlanner facade. setConfig (re-inits the Mpc)
  / setPath / setGoal / setCostmap / clearGoal / compute(current) -> MpcResult.
  compute() runs the precedence gates (NO_GOAL, NO_COSTMAP, NO_PATH), the
  reached check (xy + wrapped yaw err vs tol -> GOAL_REACHED), then generateReference
  + mpc.solve and returns TRACKING with the command + predicted on success. The
  facade owns the open-loop warm start: last_cmd (u_{-1} for the slew constraint,
  slew seed) and last_predicted (the obstacle nominal fed back into solve, warm start), both
  reset to zero/empty on every stop. A solver failure -> the new ControllerState
  STUCK with a zero command. ROS-free (takes CostmapView + std::vector<Pose2D>).
- src/local_planner_node.cpp -- thin ROS node (g1_local_planner_node). 20 Hz timer,
  AsyncSpinner(2) + one mutex. Subscribes path/costmap/goal, does the TF lookup
  off-lock (last-good reuse on miss), runs facade.compute() + facade.setConfig() both
  serialised under mu_, publishes after release. Enable gate (std_srvs/SetBool
  ~enable, default OFF): cmd_vel zeroed when disabled; local_plan + state always
  publish. DISABLING ALSO clearGoal()s: a disabled controller has no active goal,
  so compute() reports NO_GOAL instead of latching the just-finished goal's
  GOAL_REACHED -- else the coordinator (g1_nav), which re-forwards the goal on every
  re-enable, could read that stale GOAL_REACHED on the first tick after a NEW goal
  and spuriously finish (a far goal HALTs without moving). dynamic_reconfigure (cfg/LocalPlannerTuning.cfg -> LocalPlannerTuningConfig)
  updates live knobs and calls setConfig under mu_. Config: config/local_planner.yaml.
  Launch: launch/local_planner.launch (opt-in start_costmap/start_global/start_slam
  etc). The launch exposes cmd_topic + goal_topic args (standalone defaults /cmd_vel,
  /move_base_simple/goal) and applies them as <param> overrides on the node, so a
  coordinator (g1_nav) routes this node's output/goal by forwarding args -- this node
  owns the wiring, the coordinator never reaches in with a remap. RViz:
  config/local_planner.rviz. Launch contract test: test/test_local_planner_launch.py.

## Key decisions
- Prediction horizon: N=20, dt=0.1 s (2 s horizon); control loop 20 Hz (dt_control != dt_mpc).
- Obstacles = soft penalty from bilinear costmap gradient, sampled+averaged over the
  rotated FOOTPRINT circle centers (falls back to single-point when no footprint set);
  QP always feasible. Linearised about a warm-started nominal (last cycle's predicted
  path) so avoidance self-reinforces; falls back to the reference points when no
  nominal is supplied.
- Footprint hard guarantee = a post-MPC safety filter (core/footprint_safety), NOT a
  QP constraint: it verifies the applied command's swept footprint vs the costmap and
  scales/brakes so the footprint never overlaps a lethal cell. STUCK on a full block.
  Run in the facade compute() solved-branch when footprint_ok_ && safety.enabled.
- LTI linearisation about current heading theta0 (fixed over the horizon).
- Velocity limits: v_ref=0.5, max_vx=0.5, min_vx=-0.3, max_vy=0.3, max_wz=0.8 (m/s, rad/s);
  acc_lim y=1.0, theta=1.5 (m/s^2 / rad/s^2); xy_tol=0.05m, yaw_tol=0.03rad. acc_lim_x:
  code default 1.5 (MPC slew-mechanics test fixtures), deployed YAML 0.5 to match
  g1_locomotion ax_max so planned braking is executable (honest braking).
- Reference speed is regulated each cycle (referenceSpeed): curvature cap
  sqrt(a_lat_max/kappa), approach ramp sqrt(2*a_decel*remaining), heading-error factor;
  cruise terms floor at v_min_move, the approach ramp reaches 0 at the goal. New live
  knobs: a_lat_max, a_decel, curv_lookahead, heading_slow_start/full/floor, v_min_move.
- No separate terminal P-controller; one MPC controller with heading ref switching to
  goal yaw within goal_align_radius (0.5 m).
- All params have code defaults; no hardcoded values.
- Slew constraint uses last applied u_0 (open-loop self-referential).

## Known limits (live-tunable; first-motion is enable-gated + e-stop)
- Default tuning is TRACKING-dominant (w_obstacle=2 << w_pos=10, like the reference
  DWA path_distance>>occdist): the GLOBAL path is the primary obstacle avoider; the
  local obstacle term is a secondary tracking-margin + dynamic-obstacle nudge. An
  obstacle-dominant default (w_obstacle=50) stalled the tracker on inflation-grazing
  global paths (it shoved the prediction off any path touching the inflation band).
  Bump w_obstacle live for stronger local avoidance in open space.
- The warm-started obstacle nominal can limit-cycle around a STATIC head-on block (the
  prediction bows clear -> gradient ~0 there -> next solve snaps back). This is now
  DAMPED by obstacle_nominal_damping (default 0.5): the nominal is blended toward the
  reference each cycle in the facade before solve(), so it cannot snap back. Set 0 for
  the legacy undamped warm start. Narrow-corridor strafe is further suppressed near
  obstacles by vy_suppress_cost/scale (scale applied vy down when the footprint sits in
  high cost). Clean termination near a goal-by-obstacle: reach_on_path_end + a
  no-progress stall-near-end latch (reach_stall_progress/cycles). The stall near-end
  gate is reach_stall_radius (default 0.10 m), decoupled from goal_align_radius (0.5 m,
  which governs only the approach decel + heading switch), so the stall fallback latches
  only when genuinely close to the endpoint. Precision: a REACHABLE goal (path endpoint
  within reach_goal_gap, default 0.15 m, of the raw goal) is judged AND tracked against
  the RAW goal (the terminal reference retargets to it), removing the ~half-cell
  grid-snap gap of the path endpoint; an UNREACHABLE goal (endpoint far short, e.g. goal
  in an obstacle) falls back to the path endpoint. Default tolerances xy 0.05 m / yaw
  0.03 rad.
- A global path that doubles back SHARPER than the fixed-theta0 2 s LTI horizon can
  represent is not trackable by the local MPC; the global planner should not emit
  such hairpins.

## Build / test
```bash
# from catkin_ws/:
catkin build g1_local_planner && source devel/setup.bash
catkin run_tests g1_local_planner --no-deps && catkin_test_results build/g1_local_planner
```
C++ gtests (costmap view, kinematic model, path reference, mpc, facade) + a python
launch/config contract check. Integration gate: an offline closed-loop replay
(gitignored harness under tmp/g1_lp_gate; links g1_local_planner_core, builds the real
inflated costmap from a saved SLAM map via g1_costmap_core, plans free-corridor global
paths via g1_global_planner_core, drives the facade closed-loop with stepTrue at dt,
asserts reach + collision-free + within-limits, plus a synthetic obstacle corridor) +
the owner-in-the-loop RViz motion gate (2D Nav Goal -> tracked /nav/local_plan -> arm
~enable -> walk + avoid a moved obstacle -> stop aligned). Node binary
g1_local_planner_node; node name g1_local_planner.

