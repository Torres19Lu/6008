# g1_slam_frontend - agent guide

LiDAR-inertial odometry: an iterated error-state Kalman filter (iEKF) on the
manifold, refactored to fit humis-g1. In: /livox/lidar (Livox CustomMsg) +
/livox/imu. Out: /slam/frontend/odom (odom->base_link),
/slam/frontend/cloud_registered, and TF odom->base_link. The estimator core is the
bundled iEKF + incremental k-d tree map; the integration is ours.

## Boundary (do not break)
- Integration boundary is ROS-topic IPC: other packages consume this node's
  topics only; do not add a library export they link against. This
  node bundles an incremental k-d tree map (include/ikd_tree) and a Kalman-filter-
  on-manifolds toolkit (include/ikfom_toolkit); their original copyright headers in
  the source files stay intact.

## Layout
- Our headers: `include/g1_slam_frontend/{common_lib,so3_math,preprocess}.h`,
  `{imu_processing,use_ikfom}.hpp` (snake_case). Sources: `src/laser_mapping.cpp`,
  `src/preprocess.cpp`.
- Vendored deps under `include/ikd_tree/` and `include/ikfom_toolkit/`. Their
  file/dir names were conformed to snake_case (and the include paths updated); the
  C++ identifiers, types, and code content are unchanged upstream (e.g. `KD_TREE`,
  `SO3`, `S2`, `MTK`). Note: this gives up easy diff-against-upstream (accepted).
- No `msg/`: `Pose6D` is a plain struct in `common_lib.h` (estimator-internal IMU
  preintegration state; never published), so there is no message_generation dep.
- No `g1_msgs` dep (frontend uses none); deps are roscpp/sensor/geometry/nav_msgs/
  tf/tf2_ros/pcl_ros/eigen_conversions/livox_ros_driver2.

## Invariants (do not break)
- **Params live in the node's PRIVATE namespace.** main() uses
  `ros::NodeHandle nh("~")` (NOT upstream's global `nh`); the launch loads
  config/slam_frontend.yaml via `<rosparam>` INSIDE `<node>`. If these drift apart,
  every nh.param silently falls back to its CODE DEFAULT - and the default
  `extrinR(9,0.0)` is a ZERO rotation, which collapses the cloud and makes the
  estimator diverge the instant the robot moves (stable while still). Topics use absolute
  names ("/cloud_registered", "/Odometry") so they ignore this namespace; remaps
  stay in launch.
- **Orientation = gravity-align + compose inversion.** Two pieces make the map
  right-side up:
  1. IMU_init gravity-aligns the world: it sets `init_state.rot` so measured
     gravity maps to world -Z and `grav = (0,0,-G)`, so `odom` is Z-up by
     construction (cloud_registered is then right-side up). Upstream left this off
     (map in the initial IMU frame), which for the G1's inverted Mid-360 (IMU/cloud
     Z-down, acc_z ~ -1g) gives an upside-down map.
  2. The Mid-360 is **physically inverted** vs the URDF mount (cloud Z-down, URDF
     mid360_link Z-up). `config frames/cloud_to_lidar_R` (= 180 about X) reconciles
     the real cloud frame to the URDF mid360_link IN THE COMPOSE so odom->base_link
     is upright. The URDF is NOT modified. The 2.3deg pitch is honored via the TF term.
  Do NOT "fix" this by changing the lidar-IMU extrinsic: it is **identity** (IMU and
  cloud are aligned; confirmed because identity tracks and Z-flips diverge).
- **odom->base_link compose.** publish_odometry composes
  T(odom->base_link) = T(odom->mid360_real)[state + offset_R/T_L_I] *
  cloud_to_lidar_R * T(mid360_link->base_link)[live tf2 lookup]. The last term is
  consumed from TF (robot_state_publisher, g1_description); the node does NOT publish
  /joint_states or start any upstream. The launch's start_state arg brings up the real
  source: g1_locomotion (real /joint_states incl. waist_yaw, from the robot LowState) +
  g1_description (robot_state_publisher). On lookup failure it REUSES the last good
  transform (g_T_LB_cache); only before the first success does it fall back to identity,
  with a throttled warn. It broadcasts ONLY odom->base_link; do NOT restore the upstream
  camera_init->body TF broadcast (mid360_link would get two parents, breaking TF).
  HANDEDNESS: `cloud_to_lidar_R` here means `R(cloud<-mid360)`; any consumer that
  needs the opposite direction (e.g. rear_clip_filter, which maps cloud->base) MUST
  use `cloud_to_lidar_R.transpose()`, not the bare matrix. The default 180-about-X is
  involutory so a bare-matrix bug is invisible today - keep the transpose explicit so
  a future non-involutory mount rotation does not silently break one consumer. MINOR
  KNOWN ISSUE (D2): the TF is looked up at `ros::Time(0)` (latest) but composed with a
  `lidar_end_time` state and stamped at `lidar_end_time`, so under fast `waist_yaw`
  the published base_link pose (and the rear-clip sector) lead by up to ~1 scan
  (~few deg); the ESTIMATOR is unaffected (the TF is post-hoc). Fix later with a
  time-stamped lookup if aggressive waist motion matters.
- **No map->odom, no sensor-frame TF.** That is the backend. This node
  owns odom->base_link only. /slam/keyframe is handled by the backend.
- **CustomMsg input.** lidar_type=1 (avia_handler). The Mid-360 must run with
  xfer_format:=1; g1_lidar's default stays PointCloud2. Frames/topics/extrinsics
  come from config; no hardcoded values.
- **Rear follow-sector clip (humis-g1 add, ON by default).** rear_clip_filter()
  runs in the main loop AFTER sync_packages and BEFORE p_imu->Process (MAIN thread,
  so reading g_T_LB_cache is race-free - do NOT move it into the lidar callback,
  which runs in the spinner thread and would tear-read the cache). It drops points
  in a base_link sector centered on the rear (-X) AND beyond rear_clip/max_range, so
  a person following past max_range never reaches the EKF or the map. Frame is
  base_link via the SAME mid360_link->base_link TF the compose uses
  (`T_LB.inverse() * cloud_to_lidar_R.transpose()` - the transpose because
  `cloud_to_lidar_R` is `R(cloud<-mid360)` and we need `mid360<-cloud`; correct for
  ANY rotation, not just the involutory default), NOT the sensor, so waist_yaw does
  not swing the sector. Per-point test: xb <= -cos_half*hr && hr > max_range (no trig;
  cos_half cached at init). On a TF dropout it reuses g_T_LB_cache; before the first
  TF it KEEPS all points + throttled warn (needs start_state - it never silently
  drops data it cannot place). Params in rear_clip/* (enable default true); the
  launch rear_clip arg overrides enable. Disable for full surround mapping. MINOR
  KNOWN ISSUE (D3): if the lidar comes up BEFORE start_state (no TF yet), the
  startup scans keep their rear points, which enter the initial ikd-tree and persist
  for the run (fov-segment only drops points leaving the 500 m cube); bring start_state
  up with/before the lidar to avoid the startup residue.
- **No source-tree writes.** The upstream debug Log/ + PCD/ file output and the
  ROOT_DIR machinery were removed. Do not reintroduce file writes into the package.
- **C++17** (upstream C++14). matplotlib/PythonLibs and the unused OpenCV-pulling
  Exp_mat.h were dropped.

## Build / test
```bash
# from catkin_ws/:
catkin build g1_slam_frontend && source devel/setup.bash
catkin run_tests g1_slam_frontend --no-deps && catkin_test_results build/g1_slam_frontend
```
test/ is launch + config-contract checks only (ET + pyyaml); the estimator is
covered by the live gate (a real G1 Mid-360 bag). Node binary
g1_slam_frontend_node; node name g1_slam_frontend.

Design details are in the source comments in src/laser_mapping.cpp and config/slam_frontend.yaml.
