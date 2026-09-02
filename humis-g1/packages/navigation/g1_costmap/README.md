# G1 Costmap (g1_costmap) 🧱

2D **layered occupancy costmap** for the Unitree G1. It fuses the static SLAM map,
the live registered cloud, and robot-radius inflation into a single 3-state grid
(free / obstacle / unknown) the global planner can search. Obstacles are decided
by height **relative to the local ground**, not an absolute z window: each cell
has its own ground (the lowest return, with the stance-foot contact as a
fallback), so a slope reads walkable while obstacles on it still read as
obstacles. The static map is fused **probabilistically** (Bayesian log-odds across
keyframes, log-odds): a cell needs consistent evidence to read as an
obstacle, and free observations vote out a noise or drift obstacle, instead of one
stray point painting a permanent block. The ground/obstacle split is **normal-gated**
(a near-vertical surface on the floor is an obstacle; a near-horizontal patch just
above ground is drifted floor), which makes it robust to SLAM Z drift and slope.
The live cloud feeds a **bidirectional, decaying change layer**: a door mapped
closed but seen persistently open is cleared to free, and pedestrian ghosts fade
back to the map, all without ever rewriting the SLAM map file.
Implemented in standalone C++ with log-odds occupancy fusion, normal-angle ground
segmentation, and obstacle erosion.

## 🧱 What you get
- `/nav/costmap_raw` (`nav_msgs/OccupancyGrid`, `map` frame) - the **3-state
  2D grid map**: free (0) / obstacle (100) / unknown (-1), **no inflation**.
  This is the **default RViz view** (`rviz/Map`, Color Scheme `map` -> white free /
  black obstacle / gray unknown, a clean flat 2D map)
- `/nav/costmap` (`nav_msgs/OccupancyGrid`, `map` frame) - the **inflated costmap**
  for the planner: the same 3-state plus an inflation band (1..99) around obstacles.
  RViz toggle via `rviz/Map`, Color Scheme `costmap` (the move_base nav look; this
  is the inflation view)
- `/nav/local_costmap` (`nav_msgs/OccupancyGrid`, `map` frame) - a **rolling
  robot-centered crop** of the inflated master: 6 m x 6 m
  square window (60 x 60 cells at 0.1 m/cell) re-centered on `base_link` each
  timer tick; cells outside the master bounds padded as unknown (-1). Consumed by
  `g1_local_planner` as the MPC obstacle field. Configure via `local_costmap_enable`,
  `local_costmap_topic`, `local_size_m`, `robot_base_frame` in `config/costmap.yaml`
- `/nav/costmap_prob` (`nav_msgs/OccupancyGrid`) - the static fusion's per-cell
  occupancy **probability** (0..100, unknown -1, no inflation); an off-by-default
  diagnostic, render as a `Map` overlay (Scheme `map`) to inspect fusion quality
- `/nav/change_prob` (`nav_msgs/OccupancyGrid`) - the **dynamic change layer's**
  per-cell occupancy probability (0..100, -1 neutral, no inflation); an
  off-by-default diagnostic, render as a `Map` overlay to tune the add/clear
  thresholds (low where a cell is cleared/opened, high where an obstacle is added)
- Free space comes from **raycasting** the backend's per-keyframe clouds
  (`/slam/keyframes`): the swept floor between each keyframe origin and its returns
  is cleared free, plus an angular gap-fill of the unobserved wedges (a bare
  point-cloud projection would mark only sensed surfaces, leaving "punched" holes)
- A ROS-free core (`g1_costmap_core`: grid + ground surface + log-odds accumulator
  + layers + fusion) behind a thin I/O node; the integration boundary is ROS topics
- Live tuning of the band, decay, and inflation via `dynamic_reconfigure`
- A `clear_costmap` service (`std_srvs/Empty`, node private namespace) that
  immediately flushes the dynamic (transient-obstacle) layer while leaving the
  static SLAM map untouched; the next timer tick re-fuses and republishes the
  cleared master. Called by the `g1_nav` recovery FSM as the first
  (cheap, no-motion) recovery step. A still-present obstacle reappears on the
  very next dynamic update -- this clears stale/transient marks, not walls

The global grid and rolling local window (`/nav/local_costmap`) for the MPC
local planner are both produced by this node (consumed by `g1_local_planner`).

## 📦 First-time setup
- Build the workspace (see `docs/INSTALL.md`). No special third-party deps: the
  costmap uses PCL/Eigen (standard libs) and standard ROS message types.

## 🚀 Run
```bash
# SLAM (or a bag) already publishing /slam/keyframes + cloud_registered + the TF
# tree (incl. the leg TF for the foot ground reference):
roslaunch g1_costmap costmap.launch
# + RViz (costmap overlaid on the SLAM map):
roslaunch g1_costmap costmap.launch rviz:=true
# bring up the SLAM backend too (sensors/state up elsewhere):
roslaunch g1_costmap costmap.launch start_slam:=true rviz:=true
# full live stack (costmap + backend + frontend + sensor + robot state):
roslaunch g1_costmap costmap.launch start_slam:=true start_frontend:=true start_lidar:=true start_state:=true rviz:=true
```
This node only **consumes** topics + TF. Per the launch-composition convention,
opt-in args (default off) bring up the upstream: `start_slam` launches
`g1_slam_backend`, and `start_frontend` / `start_lidar` / `start_state` /
`map_path` are forwarded through it.

## 🧭 How it works
- **Ground reference (foot contact).** The node reads both `*_ankle_roll_link`
  TFs (published by `robot_state_publisher` from g1_locomotion's `/joint_states`)
  and projects the foot sole collision-sphere centres (the `foot_*` params, from
  g1_description ankle_roll) through each pose, taking the lowest map z minus the
  sphere radius as the ground level. Using the full pose makes it exact under foot
  tilt, and the min over both feet picks the lower (stance) foot. This is the
  fallback for cells the per-cell surface has not yet seen. (`base_footprint` is
  not used: it projects to odom z=0, which is the pelvis-start height, ~0.7 m above
  the true floor.)
- **Ground surface (per cell, slope-aware).** Each (x,y) cell holds the robust
  median of nearby cell-lowests (the floor is the lowest return; the median rejects
  far-below outliers), trusted after `ground_min_points` returns and filled within
  `max_fill_radius`. Built from all keyframe points.
- **Obstacle domain = ground -> lidar plane.** A return is an obstacle when it is
  above the per-cell ground walkable band AND below the **lidar plane** (the sensor
  z minus `phantom_drop`); above the lidar plane is overhead the robot walks under
  (ignored). The lidar plane is the keyframe origin z (static) or the `mid360_link`
  z (dynamic), with `obstacle_max_height` as a generous fixed safety cap. So a tall
  table below the lidar reads fully as obstacle, not partly cut at a fixed height.
- **Static layer (per-keyframe local grid -> log-odds global map).** Consumes
  `/slam/keyframes` (each keyframe: sensor origin + map-frame cloud, from the
  backend). For each keyframe it builds a local grid in **two passes**: it segments
  returns by height offset from the cell ground **refined by the surface normal** (a
  near-vertical face on the floor -> obstacle, a near-horizontal patch just above
  ground -> drifted floor/free), marks the obstacle cells, then raycasts origin ->
  each return clearing the beam FREE **but stopping at the first obstacle** (the
  occlusion guard: a beam passing over an obstacle in 3D must not clear its 2D
  footprint or the cells occluded behind it, so a table's occluded centre stays
  unknown, not free), and **angular-fills** the unobserved wedges. Each local grid
  casts one hit/miss vote per cell into a **log-odds accumulator** (so one keyframe
  cannot double-count, and noise is outvoted across viewpoints). The accumulated map
  is thresholded to 3-state, isolated obstacle speckle is **eroded**, and residual
  holes are closed. Grid sized from robust XY percentile bounds. The per-cell
  probability is published on `/nav/costmap_prob`. The stop-at-obstacle occlusion
  guard ensures the node never invents 2D free over an obstacle.
- **Dynamic change layer.** The live `cloud_registered` (odom frame) is transformed
  to `map` and classified the same way on the update timer (same ground->lidar-plane
  band, raycast origin = the `mid360_link` lidar), but instead of a binary occupancy
  it accumulates **signed log-odds change evidence** vs the static map: a raycast
  miss lowers a cell (with the **same stop-at-obstacle** occlusion guard), an
  obstacle return raises it, and every tick all evidence **relaxes toward neutral**
  by `dyn_decay_half_life`. So a door mapped closed but seen persistently open is
  voted free, a pedestrian ghost fades once it leaves, and a re-observed obstacle
  holds. A cell is voted an obstacle only when a supported fraction of its returns
  are in-band (`dyn_obstacle_frac`, default 0.4) with at least `dyn_min_returns`
  returns, so flat-ground SLAM Z scatter is not mistaken for a ring of obstacles (the
  grid-native RTAB-Map cluster-min-size analog). The SLAM map is never modified. No
  self-filter is needed (the inverted Mid-360's FOV does not reach the legs or feet).
- **Fusion + output (three-way asymmetric).** The static map and the dynamic change
  evidence combine into the 3-state occupancy by a three-way rule: evidence at/above
  `dyn_add_thr` adds an obstacle (low bar, over any static cell); evidence at/below
  `-dyn_fill_thr` fills a static UNKNOWN free (low bar); evidence at/below
  `-dyn_clear_thr` clears a static LETHAL to free (an opened door; **HIGH bar**, so a
  wall is not cleared by noise); otherwise the static cell stands. The result is
  published raw on `/nav/costmap_raw` (the 3-state grid map), then inflated (inscribed
  within `robot_radius`, an exponential falloff to `inflation_radius`; unknown stays
  unknown) into the planner costmap on `/nav/costmap`. The heavy point transform runs
  off the lock; a TF lookup miss reuses the last good transform (throttled).

## ⚙️ Tuning
All thresholds/topics/frames live in `config/costmap.yaml` (loaded into the node's
private namespace). The live-tunable subset is also exposed via
`dynamic_reconfigure` (`cfg/CostmapTuning.cfg`): `walkable_layers`,
`obstacle_max_height`, `dyn_decay_half_life`, `dyn_add_thr`,
`dyn_clear_thr`, `dyn_obstacle_frac`, `dyn_min_returns`, `inflation_radius`,
`cost_scaling_factor`. The fusion/segmentation
params (`prob_hit` / `prob_miss` / `prob_clamp_*` / `occupancy_thr`, `use_normals` /
`normal_k` / `ground_normal_angle` / `vertical_normal_angle` /
`normal_flat_grace_layers`, `erode_obstacles`, `angular_fill` /
`angular_fill_step_deg`, `static_max_range`, `phantom_drop`, `lidar_frame`) are
structural YAML (they take effect on the next keyframe rebuild, not live). The change
layer's probability params (`dyn_prob_hit` / `dyn_prob_miss` / `dyn_clamp_min` /
`dyn_clamp_max` / `dyn_fill_thr`) and the localization guard `freeze_static` are
structural YAML too. Set `freeze_static:=true` for the localization phase: the static
map is then frozen (keyframe updates ignored after the first build) and the change
layer handles live avoidance + door clearing purely in memory, never writing the map
file.

> **Gate-tune the band.** `walkable_layers` is the half-thickness of the floor band
> (in cells); the band TOP is the lidar plane (`lidar_z - phantom_drop`), so
> `phantom_drop` trims ghosts just under the sensor and `obstacle_max_height` is
> only a generous safety cap (raise it, don't rely on it as the top). Verify the
> `foot_*` sole-contact params against the g1_description ankle_roll collision and
> `lidar_frame` against the URDF. The
> ground is per-cell relative, so these are robust to where the SLAM map initialized.
>
> **Fusion knobs.** Raise `occupancy_thr` (or lower `prob_hit`) if noise still
> reads as obstacles; raise `prob_miss` (toward 0.5) if real obstacles get cleared
> too eagerly. Turn `use_normals` off to fall back to pure height bands (faster, no
> drift/slope robustness). `angular_fill` trades a denser free map for some risk of
> over-claiming free in genuinely unobserved wedges.

## 🧪 Build & test
```bash
# from catkin_ws/:
catkin build g1_costmap && source devel/setup.bash
catkin run_tests g1_costmap --no-deps && catkin_test_results build/g1_costmap
```
C++ unit tests: the height + normal-gate classifier (layer snap, 3-state band,
flat-demote / vertical-promote, foot ground), the per-cell ground surface (lowest,
robust median, min-points trust, neighbour fill), the log-odds accumulator (update,
clamp, threshold, noise vote-out), grid world<->cell, the static layer (raycast
free, keyframe log-odds vote-out, normal gate, erosion, hole fill) + the dynamic
change layer (signed log-odds hit/miss votes, decay toward neutral, band + range
gating, clear-to-neutral), inflation, and three-way asymmetric fusion (add /
fill-unknown / clear-static, ghost decay, clearDynamic). A python check guards the
launch/config contract. The full gate is an
offline replay (saved map + a `cloud_registered` bag with the leg TF) with RViz +
a moved synthetic obstacle.

See `docs/INSTALL.md` for build and environment setup.
