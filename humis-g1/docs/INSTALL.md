# humis-g1 Installation

Canonical fresh-PC setup guide (clone -> environment -> drivers -> build). Living
doc: each phase appends any new setup steps it introduces.

Prereqs: Ubuntu 20.04, ROS Noetic, conda at ~/miniconda3, NVIDIA driver + CUDA.
(catkin_tools is installed by scripts/install_system_deps.sh below.)
Export the robot network interface in ~/.bashrc:

    export G1_NETWORK_INTERFACE=wlx6c1ff7e45bf8

## 1. Clone and init submodules
    git clone <repo> humis-g1 && cd humis-g1
    git submodule update --init --recursive

## 2. System dependencies
    ./scripts/install_system_deps.sh        # catkin_tools, GTSAM, OSQP, osqp-eigen
    # PCL/Eigen/OpenCV/yaml-cpp come from ros-noetic-desktop-full
    sudo apt install -y ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui   # g1_description display.launch
    sudo apt install -y python3-nose   # g1_lidar / g1_slam_frontend / g1_slam_backend / g1_costmap / g1_global_planner / g1_local_planner / g1_nav catkin run_tests (nosetests + rostest)
    sudo apt install -y python3-yaml   # g1_slam_frontend / g1_slam_backend / g1_costmap / g1_global_planner / g1_local_planner / g1_nav launch/config test

## 3. Third-party builds
    (cd external/Livox-SDK2 && mkdir -p build && cd build && cmake .. && make -j && sudo make install && sudo ldconfig)
    (cd external/livox_ros_driver2 && cp package_ROS1.xml package.xml)
    (cd external/unitree_sdk2 && mkdir -p build && cd build && cmake .. && make -j && sudo make install)

## 4. Conda env (YOLO sidecar)
    source ~/miniconda3/etc/profile.d/conda.sh
    conda env create -f config/conda/humis_g1_env_py310.yml

## 5. Build the workspace
    ./scripts/link_workspace.sh
    source /opt/ros/noetic/setup.bash
    cd catkin_ws
    catkin config --cmake-args -DROS_EDITION=ROS1   # required by livox_ros_driver2
    catkin build
    source devel/setup.bash

## 6. Fetch map assets (g1_maps, optional)
Map binaries (`*.pcd` / `*.g2o` / `*.bin`) are not in git; pull them from the private
HF dataset and verify each sha256 (needs a read token: `hf auth login` or `HF_TOKEN`):

    python3 packages/mapping/g1_maps/scripts/fetch_maps.py   # reads packages/mapping/g1_maps/config/maps_manifest.yaml

Idempotent (skips files already present with the right sha256). Required only to run
the map-management demos/tests against the sample map; not needed to build.

## Notes
- `g1_slam_frontend` is a port of FAST-LIO2 (it bundles ikd-Tree + IKFoM_toolkit);
  the port is self-contained in-tree, so no extra third-party build is needed. The
  upstream authors' copyright headers stay in the source files. The repo is
  private/internal-use-only.
- `g1_slam_backend` reimplements Scan Context + a GTSAM iSAM2 pose graph against
  GTSAM as a standard library (`find_package(GTSAM)`; the ROS-distributed GTSAM 4.2,
  already pulled by `install_system_deps.sh`), so it bundles no third-party source.
  It derives keyframes from the front-end odom + cloud; the front end is untouched.
- `g1_costmap` is a clean own-C++ 2D layered costmap (static SLAM-map projection +
  dynamic live-cloud layer + inflation); it depends only on standard libs (PCL/Eigen)
  and standard ROS messages, so it bundles no third-party source. `references/costmap_2d`
  is study-only and removed at the reference-cleanup checkpoint.
- `g1_global_planner` is a clean own-C++ A* 2D global planner (8-connected, octile
  heuristic, inflation-aware cost, line-of-sight smoothing); it reads `/nav/costmap`
  as a standard `nav_msgs/OccupancyGrid` (it does NOT link `g1_costmap`) and depends
  only on standard ROS messages, so it bundles no third-party source.
- `g1_local_planner` is a clean own-C++ linear MPC local tracker (a Quadratic Program
  per 20 Hz control cycle) solved with OSQP + osqp-eigen (installed by
  `scripts/install_system_deps.sh`); it reads `/nav/global_path` + `/nav/local_costmap`
  as standard `nav_msgs` messages (it does NOT link `g1_costmap_core` or
  `g1_global_planner_core`) and emits `/cmd_vel` as a standard `geometry_msgs/Twist`,
  so it bundles no third-party source.
- `g1_nav` is a thin own-C++ navigation FSM orchestrator: it serves the `navigate_to`
  actionlib action, is the sole `/cmd_vel` writer (a velocity mux over the local
  planner's `/nav/cmd_vel_track`), runs a lean recovery sequence (clear / rotate /
  back-up / wait), and coordinates the already-separate planner nodes over ROS-topic /
  action IPC. It depends on `g1_msgs` (the `NavigateTo` action) plus, at runtime, the
  planners and `g1_locomotion`; it reads the standard planner state / TF and does NOT
  link `g1_costmap_core`, `g1_global_planner_core`, or `g1_local_planner_core`, so it
  bundles no third-party source. A ROS-free core (`g1_nav_core`: FSM + recovery
  sequencer + progress watchdog) sits behind the thin node.
- `references/FAST_LIO` (git-ignored) is for study/diffing only and is not required
  to build; delete it once it is no longer needed for study. The SLAM-backend
  study clones (`references/{LIO-SAM,SC-LIO-SAM,FAST_LIO_SLAM,scancontext}`) are
  likewise git-ignored, reference-only, and can be deleted when no longer needed.
