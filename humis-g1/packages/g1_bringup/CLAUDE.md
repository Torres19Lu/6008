# g1_bringup - agent guide

Top-level bring-up / integration package (composition root) for the humis-g1 stack.
RESERVED PLACEHOLDER: builds clean and holds the name; NOT yet implemented.

## Status (reserved placeholder)
Empty by design. Only package.xml + CMakeLists.txt + README.md + CLAUDE.md exist.
No launch/, scripts/, or config/ yet. find_package(catkin) + catkin_package() with no
targets -> configures and builds green, reserving the slot. Do NOT claim or
add bring-up behavior that is not actually here; fill scope only when a concrete plan exists.

## Intended scope (future, do not pre-build without a plan)
- One-command bring-up of the full stack (description/TF, lidar, locomotion, SLAM
  frontend+backend, costmap, global+local planner, g1_nav, perception).
- Scenario launches for the end-to-end acceptance gate (map -> "go to the chair" /
  nav-point -> avoid dynamic obstacle -> relocalize after restart).
- Launch the YOLO sidecar (g1_yolo_sidecar) as a conda subprocess in
  humis_g1_env_py310 -- a non-ROS subprocess, not a ROS node.

## Conventions to follow when implementing
- Launch composition: downstream launches expose opt-in args (default false) to
  bring up needed upstreams; never fabricate state not started.
- No hardcoded values: paths / IPs / params come from config / args / env (PC2,
  network interface, map_path are env- or arg-driven, never literals in code).
- Package docs: keep this CLAUDE.md + README.md in sync as scope fills.
- Authorship / license: Proprietary, owner identity.

## Build
```
catkin build g1_bringup && source devel/setup.bash
```
