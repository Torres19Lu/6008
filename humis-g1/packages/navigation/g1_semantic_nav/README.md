# Minimal semantic navigation for G1 simulation and hardware

This package connects the already-running realtime VLM and SLAM pipeline to the
existing G1 navigation stack. It does not duplicate SLAM, costmap, global-planner,
local-planner, or `g1_nav` logic. The realtime VLM target-localization algorithm
and output contract are unchanged; only its RGB-D input adapter is extended to
accept both Gazebo and RealSense depth formats.

## Responsibilities

- `semantic_goal_adapter.py` waits for a stable `/vlm/semantic_target`, reads the
  object coordinate from `/vlm/target_pose`, samples free approach poses on
  `/nav/costmap`, and sends `GOAL_POSE` goals to `/navigate_to`.
- `sim_cmd_vel_bridge.py` applies the sole `/cmd_vel` output from `g1_nav` to the
  Gazebo `g1_sim` model and republishes `/simulation/ground_truth_odom` for the
  existing simulated Livox bridge.
- `semantic_navigation_sim.launch` starts only navigation-side components. Gazebo,
  SLAM and `slam/ros_vlm_node.py` must already be running.

The adapter first tries a point 1.2 m from the target on the target-to-robot ray.
If that cell or route is blocked, it tries the remaining free points around the
target. Every pose faces the target. VLM continues to publish the object position;
it does not choose a navigation endpoint.

## Build

From `humis-g1`:

```bash
./scripts/link_workspace.sh
cd catkin_ws
catkin build g1_semantic_nav
source devel/setup.bash
```

## Run in simulation

The shortest path is the repository-level one-command launcher:

```bash
cd /path/to/6008
VLFM_DIR="/path/to/vlfm" bash ./run_red_chair_navigation.sh
```

It performs an incremental build, starts Gazebo, SLAM, YOLO, OpenCLIP, realtime
VLM and this navigation package, and shuts down every child process on `Ctrl-C`.
The default target is already `red chair`; no interactive target input is needed.

To skip the incremental build after the first successful run:

```bash
bash ./run_red_chair_navigation.sh --skip-build
```

The components can still be started separately when debugging:

First start the existing Gazebo + SLAM + realtime VLM flow. In a second terminal:

```bash
source humis-g1/catkin_ws/devel/setup.bash
roslaunch g1_semantic_nav semantic_navigation_sim.launch
```

Expected flow:

```text
/vlm/target_pose + /vlm/semantic_target
  -> semantic_goal_adapter
  -> /navigate_to (g1_msgs/NavigateToAction)
  -> g1_nav -> /cmd_vel
  -> sim_cmd_vel_bridge -> Gazebo
```

The simulation launch sets `g1_nav/auto_arm=false`; the real robot launch must keep
the normal hardware arm/halt safety behavior and must not start the simulation
bridge.

## Run on a real G1

Real-hardware support uses a separate entry point so the Gazebo bridge can never
be started accidentally:

```bash
cd /path/to/6008
bash ./run_g1_red_chair_navigation.sh \
  --map-path /path/to/saved_real_map \
  --interface eth0 \
  --camera-xyz "MEASURED_X MEASURED_Y MEASURED_Z" \
  --camera-rpy "MEASURED_ROLL MEASURED_PITCH MEASURED_YAW"
```

The real launch starts D435, Livox, G1 state/description, SLAM localization,
costmaps, planners, `g1_nav` and this adapter. It does not start Gazebo or
`sim_cmd_vel_bridge.py`. `start_state=true` already starts `g1_locomotion`, so the
real launch deliberately passes `start_locomotion=false` to avoid a duplicate node.

Safety gates on the first run:

- `g1_locomotion` boots disarmed;
- the script never changes the G1 FSM;
- `semantic_goal_adapter` starts disabled;
- the operator must verify VLM/costmap/TF, put the robot in balanced walk mode,
  arm velocity streaming, and explicitly enable semantic navigation:

```bash
rosservice call /g1/arm
rosservice call /semantic_goal_adapter/enable "data: true"
```

Software soft-stop:

```bash
rosservice call /g1/halt
```

The script requires a saved map and measured camera extrinsics. These values are
intentionally not guessed. A physical e-stop and an owner-supervised low-speed
acceptance test remain mandatory.

## Shared simulation/real sensor contract

`slam/ros_vlm_node.py` now reads camera topics and model-service URLs from ROS
parameters. Its depth adapter accepts both:

- Gazebo `32FC1` depth in metres;
- RealSense `16UC1` depth in millimetres, converted with the configurable
  `~uint16_depth_scale` (default `0.001`).

The VLM output contract remains unchanged, so costmap and semantic navigation do
not need to know whether the coordinate came from simulation or hardware.
