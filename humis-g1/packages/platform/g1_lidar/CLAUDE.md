# g1_lidar - agent guide

Thin wrapper around `livox_ros_driver2` for the Livox Mid-360. No C++/ROS-node
source of its own: a Python launch-time config generator, a launch file, configs,
and tests. Downstream: `g1_slam_frontend` consumes `/livox/lidar` + `/livox/imu`.

## Invariants (do not break)
- **Host IP is resolved at LAUNCH, not build time.** The generator
  `scripts/generate_livox_config.py` reads `$G1_NETWORK_INTERFACE` (or `host_ip:=`),
  substitutes `@HOST_IP@` in `config/MID360_config.json.template`, and writes the
  git-ignored `config/MID360_config.json`. Do NOT reintroduce CMake-configure-time
  generation at CMake configure time, which would force a rebuild per IP change.
- **Generator stdout contract:** the script prints ONLY the output path on stdout;
  all diagnostics go to stderr; non-zero exit on failure. `lidar.launch` captures
  that stdout via `<param name="user_config_path" command=...>`, which roslaunch
  runs to completion before nodes start (so there is no generator/driver race).
  Any stray stdout would corrupt `user_config_path`.
- **Both `--interface` and `--host-ip` are `nargs="?"`** so an empty roslaunch
  substitution (`$(optenv G1_NETWORK_INTERFACE)` or `$(arg host_ip)` being empty)
  yields `""` instead of making argparse swallow the next flag. Keep this if you
  touch the arg parser.
- **LiDAR IP is fixed** at `192.168.123.120` (`lidar_configs[0].ip` in the
  template); it does not vary across hosts. Only the four `host_net_info` fields
  are placeholders.
- **`xfer_format=0`** -> `/livox/lidar` is `sensor_msgs/PointCloud2` (the
  point-cloud interface contract). The cloud frame is `mid360_link` (the `frame_id`
  param); the IMU frame is hardcoded `livox_frame` in livox_ros_driver2
  (lddc.cpp:481) and the param does not affect it. The Mid-360 lidar-IMU extrinsic
  is a fixed Livox calibration constant passed directly to the SLAM estimator (not
  TF), so the IMU frame need not join the URDF TF; do not patch the submodule
  (used-as-is tier).
- The generated `config/MID360_config.json` is git-ignored (`config/.gitignore`)
  and excluded from install; never commit or install it. `$(find g1_lidar)` is the
  source dir in the devel workspace, so the generator writes there at launch.
- **Console flood is filtered downstream, not in the submodule.** Livox-SDK2's
  spdlog console logger spams stdout with per-packet INFO under ROS1
  (`livox_ros_driver2` calls `DisableLivoxSdkConsoleLogger()` only under
  `#ifdef BUILDING_ROS2`, `lds_lidar.cpp`; that SDK switch is all-or-nothing and
  would also drop genuine SDK warnings/errors, which never hit `/rosout`). So the
  driver node runs under a launch-prefix `scripts/filter_node_log.sh` that drops
  ONLY the `driver_log_filter` denylist (default `Handle detection data|Receive
  Command: Id`), keeping warnings/errors/ROS messages. The wrapper `exec`s the
  node (it stays the PID roslaunch tracks - signals + exit code preserved for
  `required="true"`) and filters via process-substitution greps. On by default
  (`quiet_driver:=true`), inherited by downstream composing launches; do NOT
  "fix" this by patching the submodule. The pattern is a code default the launch
  overrides (env `G1_LIDAR_LOG_FILTER`); install it to `share/scripts` so the
  `$(find g1_lidar)/scripts/...` path resolves in an installed space too.

## Build / test / run
```bash
# from catkin_ws/:
catkin build g1_lidar && source devel/setup.bash
catkin run_tests g1_lidar --no-deps                    # registered nosetests (needs python3-nose)
# from this package dir:
(cd test && python3 -m unittest discover -v)  # fast unit/launch checks, stdlib only
roslaunch g1_lidar lidar.launch [rviz:=true] [host_ip:=A.B.C.D]
```

## Live gate (needs the powered LiDAR)
`rostopic hz /livox/lidar` (~10 Hz, mid360_link), `rostopic hz /livox/imu`
(~200 Hz, livox_frame); cloud renders in RViz.
Verified live: cloud 10.0 Hz, IMU 200 Hz, driver connects to 192.168.123.120.
