# G1 SLAM simulation and realtime VLM integration

This directory contains the SLAM-side deliverables for the Gazebo red-chair
demonstration.

## Contents

- `g1_simulation/`: Unitree G1 Gazebo model, classroom world, simulated D435,
  Livox-compatible lidar/IMU bridge, SLAM configuration and motion scripts.
- `sim_classroom/`: saved `g1_slam_backend` map with 339 keyframes.
- `ros_vlm_node.py`: realtime RGB-D semantic localizer. It subscribes to live
  ROS topics and publishes the detected target in the `map` frame.
- `run_realtime_vlm.sh`: single-terminal launcher for Gazebo, SLAM frontend,
  SLAM backend, YOLOv7, OpenCLIP and the realtime VLM node.

## Realtime interface

Inputs:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
/tf
/tf_static
```

Outputs:

```text
/vlm/semantic_target
/vlm/target_pose
/vlm/target_marker
/vlm/detection_image
```

## Launch

Build/link `g1_simulation` in the ROS Noetic catkin workspace first. If the
runtime project is not at the default path, export it before launching:

```bash
export HUMIS_G1_ROOT=/path/to/humis-g1
export VLFM_DIR=/path/to/vlfm
./run_realtime_vlm.sh
```

At the menu, press `v` to open the detection image, `d` to reset and run the
chair-approach demo, and `q` to stop every managed process.

The 20.9-second ROS bag is not stored in this Git repository because it is
larger than GitHub's normal 100 MB per-file limit.
