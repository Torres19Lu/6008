# G1 SLAM Backend (g1_slam_backend) 🗺️

Loop-closing SLAM back end for the Unitree G1: a keyframe **pose graph** that turns
the front end's smooth-but-drifting odometry into a globally consistent estimate.
It detects revisits with **Scan Context**, verifies them with **ICP**, optimizes
the graph incrementally with **GTSAM iSAM2**, and publishes the `map->odom`
correction plus a loop-corrected global map. Reimplemented from the published
algorithms (Scan Context; iSAM2 via GTSAM): Scan Context loop detection +
iSAM2 (GTSAM) optimization, treating the front end as an untouched black box.

## 🗺️ What you get
- TF `map->odom` (this node's only TF; the loop-corrected correction layer, REP-105)
- `/slam/map` (`sensor_msgs/PointCloud2`, latched, `map` frame) - the optimized global map
- `/slam/pose_graph` (`visualization_msgs/MarkerArray`) - nodes, odometry edges, loop edges
- Persistence + relocalization services: `/slam/save_map`, `/slam/load_map`
  (`g1_msgs/SaveMap`, `g1_msgs/LoadMap`), `/slam/relocalize` (`g1_msgs/Relocalize`)

## 📦 First-time setup
- Build the workspace (see `docs/INSTALL.md`); `g1_msgs` must be built first.
- GTSAM is a system library, resolved via `find_package(GTSAM)` (the ROS-distributed
  GTSAM 4.2 on this stack).

## 🚀 Run
```bash
# front end already up (or playing a bag through the frontend):
roslaunch g1_slam_backend slam_backend.launch
# + RViz (map + pose graph):
roslaunch g1_slam_backend slam_backend.launch rviz:=true
# bring up the frontend too (sensors/state up elsewhere):
roslaunch g1_slam_backend slam_backend.launch start_frontend:=true rviz:=true
# full live stack (backend + frontend + sensor + robot state):
roslaunch g1_slam_backend slam_backend.launch start_frontend:=true start_lidar:=true start_state:=true rviz:=true
# load a saved map and localize against it:
roslaunch g1_slam_backend slam_backend.launch map_path:=/path/to/map rviz:=true
```
This node only **consumes** the front-end topics. Per the launch-composition
convention, opt-in args (default off) bring up the upstream: `start_frontend`
launches `g1_slam_frontend`, and `start_lidar` / `start_state` are forwarded to it.

## 💾 Save / load / relocalize
Build a map by mapping, save it, then later load it and localize against it.
```bash
# after mapping a loop, dump an inspectable, re-optimizable map directory:
rosservice call /slam/save_map "path: '/path/to/map'"
# later (or via map_path at launch): load it and switch to localization-only:
rosservice call /slam/load_map "path: '/path/to/map'"
# force a relocalization (no guess = global Scan Context; with a guess = ICP at it):
rosservice call /slam/relocalize "{initial_guess: {orientation: {w: 1.0}}, use_guess: false}"
```
- **Map directory** (`<map>/`): `manifest.yaml`, `pose_graph.g2o` (optimized poses +
  odometry/loop edges, `g2o_viewer`-readable), `scan_context.bin` (descriptors), and
  `keyframes/NNNNNN.pcd` (per-keyframe `base_link` clouds). Saved atomically (temp
  dir + swap); inspectable with standard tools and re-optimizable on load.
- **Localization-only after load.** The loaded graph is frozen (no new keyframes).
  Relocalization runs on its own timer (`reloc/rate`), decoupled from the scan rate.
  The node **acquires** a lock with a global Scan Context query (the top
  `reloc/num_candidates` candidates are ICP-verified), then **tracks** with local
  scan-to-map ICP against a cached submap to absorb odom drift; a run of failures
  drops the lock and re-acquires. `map->odom` is **not broadcast until the first
  lock** (the live data floats until relocalized).
- **Robust acceptance.** A match is accepted by geometric **inlier ratio** (fraction
  of live points within `reloc/inlier_dist` of the map, must clear `reloc/min_inlier`),
  not by ICP fitness alone - this rejects the wrong-but-low-fitness alignments that
  appear in complex multi-feature scenes.
- **Robust tracking correction.** The per-tick scan-to-map ICP result is **not
  applied raw** to `map->odom`: near a dominant planar surface (a table/monitor the
  robot faces at close range) the ICP is geometrically under-constrained and slides,
  while the front-end odom stays smooth. The correction is (1) **projected** off the
  unobservable DOFs of the registration point-to-plane Hessian (`reloc/obs_eig_floor`
  / `reloc/obs_eig_ratio`, LIO-SAM-style degeneracy handling), (2) **clamped** per
  tick (`reloc/max_step_*`), (3) **low-passed** by `reloc/corr_gain`, and (4)
  **rejected** (coast on the smooth odom) when the raw jump exceeds
  `reloc/reject_*`. This keeps `map->odom` from teleporting the robot's map pose
  even while it stands still. Setting the floor/clamp/reject to 0 and `corr_gain` to
  1.0 restores the legacy raw-overwrite behavior.
- **`/slam/relocalize`**: `use_guess=false` runs the global acquire; `use_guess=true`
  runs guided ICP at `initial_guess` (a `map`-frame pose). The tracking submap radius
  is `reloc/search_radius`.

## 🧭 How it works
- **Keyframes (derived here, not from the front end).** The backend subscribes to
  `/slam/frontend/odom` + `/slam/frontend/cloud_registered`, time-syncs them, and
  selects keyframes by distance/angle. Each keyframe stores its `odom->base_link`
  pose `P_i` and its cloud pulled into the `base_link` frame (`C_base = P_i^-1 *
  cloud_world`), so re-projecting at the optimized pose rebuilds the map and the
  stored coordinates stay local.
- **Loop closure.** Per keyframe: a Scan Context descriptor (built in a yaw-only
  gravity-aligned frame so gait roll/pitch does not corrupt the height bins) -> a
  ring-key + descriptor query for a revisit -> PCL ICP geometric verification ->
  a loop `BetweenFactor`.
- **Robust loop closure (two layers).** Scan Context produces FALSE loops in
  corridors (it is translation-invariant along the axis and yaw-invariant, so it
  matches opposite-heading passes) and repetitive rooms; one bad loop in a plain
  graph twists the whole map. (1) A **front-end consistency gate** rejects a loop
  whose ICP measurement disagrees with the odometry-chained relative pose
  (`P_cand^-1 * P_id`) beyond a drift budget (`loop/max_dt + rate*path`) - the
  front end is accurate, so a large disagreement means a false/degenerate match.
  (2) Surviving loops are added under a **Cauchy robust kernel** (`loop/robust_c`)
  so an outlier is down-weighted by its residual instead of distorting the graph
  (Cauchy M-estimator on each loop factor). Loop sigmas are intentionally looser than
  odom; trust is controlled by the kernel, not a tight sigma.
- **Optimization.** GTSAM iSAM2 over odometry + loop factors. The correction is
  `T(map->odom) = X_n * P_n^-1` from the latest keyframe, re-broadcast at a fixed
  rate with a fresh `now()` stamp (so the slowly-updated parent never trips
  `TF_REPEATED_DATA`).
- **No front-end feedback.** `odom` stays smooth and continuous; `map->odom`
  absorbs the loop-closure jumps (REP-105).

## ⚙️ Tuning
All thresholds/topics/frames live in `config/slam_backend.yaml` (loaded into the
node's private namespace). Tune `sc/*` (rings, sectors, range, `sc_dist_thresh`)
and `icp/*` (`max_corr_dist`, `fitness_thresh`, `loop_voxel` - the submap voxel
applied before loop ICP) per environment. For localization
tune `reloc/*` (`search_radius`, `lost_streak`, `acquire_stride`). If loops still
distort the map (corridors, repeated rooms), tighten the `loop/*` gate floors
(`max_dt`, `max_dr`); if a real long-range loop is rejected, loosen them or raise
`max_dt_rate`. `loop/robust_c <= 0` disables the Cauchy kernel (plain Gaussian).

## 🧪 Build & test
```bash
# from catkin_ws/:
catkin build g1_msgs g1_slam_backend && source devel/setup.bash
catkin run_tests g1_slam_backend --no-deps && catkin_test_results build/g1_slam_backend
```
C++ unit tests: GTSAM/PCL ABI smoke, pose-graph drift correction + Cauchy
robust-kernel outlier rejection, Scan Context retrieval/rejection, a synthetic
closed-loop end-to-end + consistency-gate rejection, map save/load round-trip, and
relocalization (global acquire + local tracking). A python check guards the
launch/config contract. The full live gate is a real G1 loopy bag (see the plan).

