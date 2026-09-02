# G1 SLAM Frontend (g1_slam_frontend) 🛰️

LiDAR-inertial odometry for the Unitree G1: an **iterated error-state Kalman
filter** adapted to the humis-g1 interfaces. It fuses the Mid-360 cloud + built-in IMU
into a smooth, continuous `odom` estimate and publishes the robot pose as
`odom->base_link`, composed through the live `waist_yaw` joint.

## 🛰️ What you get
- `/slam/frontend/odom` (`nav_msgs/Odometry`, `odom`->`base_link`)
- `/slam/frontend/cloud_registered` (`sensor_msgs/PointCloud2`, `odom` frame)
- `/slam/frontend/path` (`nav_msgs/Path`, `odom` frame; RViz trajectory)
- TF `odom->base_link` (this node's only TF; it does NOT publish `map->odom`)

## 📦 First-time setup
- Build the workspace (see `docs/INSTALL.md`); `g1_msgs` and `livox_ros_driver2`
  must be built first.
- The Mid-360 must publish Livox `CustomMsg` (per-point timestamps for de-skew),
  so the SLAM path runs `g1_lidar` with `xfer_format:=1` (its default stays
  PointCloud2 for the rest of the stack).

## 🚀 Run
```bash
# everything already up (or playing a bag recorded with xfer_format:=1):
roslaunch g1_slam_frontend slam_frontend.launch
# bring up the Mid-360 (CustomMsg) + RViz, robot state up elsewhere:
roslaunch g1_slam_frontend slam_frontend.launch start_lidar:=true rviz:=true
# full live stack (sensor + robot state/TF):
roslaunch g1_slam_frontend slam_frontend.launch start_lidar:=true start_state:=true rviz:=true
# keep rear far returns (disable the default follow-sector clip, see below):
roslaunch g1_slam_frontend slam_frontend.launch start_lidar:=true start_state:=true rear_clip:=false
```
This node only **consumes** topics/TF. The launch has opt-in args (default off) to
bring up the upstreams it needs:
- `start_lidar` -> `g1_lidar` in CustomMsg mode (`/livox/lidar` + `/livox/imu`).
- `start_state` -> `g1_locomotion` (real `/joint_states` incl. `waist_yaw`, from the
  robot's low-level state) **and** `g1_description` (`robot_state_publisher` -> the URDF
  TF tree). The two are coupled: `robot_state_publisher` needs the joint source.

The `odom->base_link` compose looks up `mid360_link->base_link` from that TF. If it is
unavailable the compose **reuses the last good transform**, falling back to identity
(`base_link` == sensor) only until the first one is seen, with a throttled warning.

## 🌳 Frames (the waist-yaw compose point)
The estimator tracks the Mid-360 sensor pose, so it cannot publish
`odom->base_link` with a static extrinsic. This node computes
`T(odom->base_link) = T(odom->mid360_link) * cloud_to_lidar_R * T(mid360_link->base_link)_live`,
reading the last term live from TF (`robot_state_publisher`, so `waist_yaw` is
accounted for), and broadcasts only `odom->base_link`. It deliberately does NOT
broadcast a sensor-frame TF (that would give `mid360_link` two parents and break
the tree). The Mid-360 lidar-IMU extrinsic is a config constant (design 6.3).

### Right-side-up world (gravity-align + inverted-mount fix)
Two things keep the map upright:
- **Gravity-align at IMU init:** the world frame is set so gravity points to `odom`
  -Z, making `odom` Z-up by construction (so `cloud_registered` is right-side up).
  The base estimator leaves the map in the initial IMU frame, which is upside down
  for the G1's Mid-360 (its IMU/cloud Z points down, `acc_z ~ -1g`).
- **`cloud_to_lidar_R` (config):** the Mid-360 is physically inverted vs the URDF
  mount (cloud frame Z-down, URDF `mid360_link` Z-up), so a `180deg-about-X`
  reconciliation in the compose makes `odom->base_link` upright. The URDF is NOT
  modified; the 2.3deg mount pitch is honored via the TF term. The lidar-IMU
  extrinsic stays identity (IMU and cloud are aligned).

## 🚷 Rear follow-sector clip (on by default)
A person following the robot would otherwise be scanned into the odometry and the
map. Before the estimator, this node drops returns that fall in a **base_link-frame
sector centered on the rear (`-X`)** AND **beyond `max_range`** (defaults: 120 deg
wide, 2 m). The near rear (`< 2 m`) is still mapped, but anyone trailing past 2 m is
removed. The sector is pinned to the **walking direction** (`base_link` via the URDF
TF), not the sensor, so turning `waist_yaw` does not swing it off the follower.
- Controlled by `rear_clip/{enable,sector_deg,max_range}` (config) and the
  `rear_clip:=true|false` launch arg. **On by default**; `rear_clip:=false` keeps the
  rear far returns (full 360 deg mapping).
- Needs the `base_link` TF, so run with `start_state` (or a bag carrying
  `/joint_states` + `robot_state_publisher`). Without that TF it **no-ops** (keeps all
  points) with a throttled warning - it never silently drops data it cannot place.
- Trade-offs: a rear map hole past `max_range`, and a small Scan-Context loop-closure
  effect in the backend (the descriptor loses that rear wedge). Acceptable for the
  follow use case; disable it for full surround mapping.

## ⚙️ Config
`config/slam_frontend.yaml` (Mid-360 estimator profile + the `frames` block + the
`rear_clip` block). Topic names are remapped to the `/slam/frontend/*` contract in
the launch file.

## ✅ Test
```bash
(cd test && python3 -m unittest discover -v)   # from this package dir: launch/config contract, stdlib + pyyaml
catkin run_tests g1_slam_frontend --no-deps                     # from catkin_ws/ (needs python3-nose)
```
The estimator is validated at the 🤖 live gate (a real G1 Mid-360 bag): see the
plan's live-gate section.

The waist-yaw compose, lidar-IMU extrinsic, and gravity-align design are described
in the source comments in `src/laser_mapping.cpp` and `config/slam_frontend.yaml`.
