# G1 LiDAR (g1_lidar) 📡

Livox Mid-360 bring-up for the Unitree G1. Wraps the `livox_ros_driver2`
submodule and publishes the point cloud + IMU in the robot's `mid360_link`
frame, with the PC1 host IP injected at launch (no rebuild on IP change).

## 📡 What you get
- `/livox/lidar` (`sensor_msgs/PointCloud2`, frame `mid360_link`)
- `/livox/imu` (`sensor_msgs/Imu`, frame `livox_frame` - driver default)

## 📦 First-time setup
- Build the workspace (see `docs/INSTALL.md`); `livox_ros_driver2` must be built.
- Export your robot network interface: `export G1_NETWORK_INTERFACE=<iface>`.
- Power the Mid-360 and connect PC1 to the G1 network (the `192.168.123.0/24`
  subnet). LiDAR IP is fixed at `192.168.123.120`.

## 🚀 Run
```bash
roslaunch g1_lidar lidar.launch              # bring up the LiDAR
roslaunch g1_lidar lidar.launch rviz:=true   # + RViz on the cloud
roslaunch g1_lidar lidar.launch host_ip:=192.168.123.42   # manual host-IP override
roslaunch g1_lidar lidar.launch quiet_driver:=false       # show the raw SDK console flood
```
The host IP is resolved from `$G1_NETWORK_INTERFACE` and written into the
git-ignored `config/MID360_config.json` from the tracked template.

## 🤫 Quiet driver logging
The Livox-SDK2 console logger (spdlog) floods stdout with per-packet INFO lines
(`Handle detection data...`, `Receive Command: Id...`). `livox_ros_driver2`
disables it only under `#ifdef BUILDING_ROS2`, and the SDK switch is all-or-
nothing - it would also hide genuine SDK warnings/errors, which never reach
`/rosout`. So `lidar.launch` runs the driver under a launch-prefix
(`scripts/filter_node_log.sh`) that drops only the denylisted lines; warnings,
errors, and all ROS messages still print. This is on by default and is inherited
by any downstream launch that composes `lidar.launch` (e.g. the SLAM frontend/
backend `start_lidar:=true`).
- `quiet_driver` (default `true`): set `false` to see the unfiltered SDK flood.
- `driver_log_filter` (extended regex, default `Handle detection data|Receive
  Command: Id`): widen it to silence more lines.

## 🌳 Frames
The cloud is stamped `mid360_link`, which hangs off `torso_link` in the
`g1_description` URDF. The IMU is stamped `livox_frame` (the driver hardcodes
it); the Mid-360 lidar-IMU extrinsic is a fixed Livox calibration constant passed
directly to the SLAM estimator, not TF, so that frame need not be in the URDF. To view the cloud on the robot
model, also run `roslaunch g1_description display.launch` and set the RViz fixed
frame to `base_link`.

## ✅ Test
```bash
(cd test && python3 -m unittest discover -v)   # from this package dir: fast, stdlib only
catkin run_tests g1_lidar --no-deps                      # from catkin_ws/: via catkin (needs python3-nose)
```

