# G1 Bring-up and Integration (g1_bringup) 🚀

**Top-level composition root** for the humis-g1 stack: a single-command bring-up of
the full autonomy pipeline plus the scenario launches that drive the end-to-end
acceptance gate. This package is currently a **reserved placeholder** -- the package
builds clean and holds the name, but no launch files, scripts, or config are
implemented yet.

## 🚧 Status: reserved placeholder

Empty by design. Only `package.xml`, `CMakeLists.txt`, `README.md`, and `CLAUDE.md`
exist; `catkin_package()` declares no targets. The package configures and builds so
the slot is reserved and `catkin build` stays green, but it brings up nothing yet.
The concrete implementation is not yet started.

## 🎯 Intended scope (future)

- **One-command bring-up** of the full stack (description/TF, lidar, locomotion, SLAM
  frontend + backend, costmap, global + local planner, `g1_nav`, perception).
- **Scenario launches** for the end-to-end gate: build a map, "go to the chair" / a
  nav point, avoid a dynamic obstacle, relocalize after restart.
- **YOLO sidecar as a conda subprocess** (`humis_g1_env_py310`): `g1_bringup` launches
  `g1_yolo_sidecar` as a non-ROS subprocess, not a ROS node.
- **Launch composition**: downstream launches
  expose opt-in args (default false) to bring up the upstreams they need; the
  composition root wires them together without fabricating state it does not start.

## 🧪 Build

```bash
# from catkin_ws/:
catkin build g1_bringup && source devel/setup.bash
```

No tests yet (nothing to test). The package depends only on `catkin` until the
concrete bring-up is implemented; runtime dependencies on the stack packages get
declared when the launches land.
