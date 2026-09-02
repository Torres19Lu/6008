# G1 Map Manager (g1_map_manager) 🗺️

**Map management + map-aware navigation layer** for the Unitree G1. It gives every map
a name and immutable version snapshots, a captured cross-map topology, and a
single navigation entry whose goals carry a map identity: same-map goals run as one
`g1_nav` leg, cross-map goals route over the gateway graph and switch maps mid-run with
gateway-seeded relocalization. It authors keyframe-anchored nav points during mapping,
edits maps offline without touching the snapshot, and never modifies a map file while
localizing. This is a THIN COORDINATOR: it orchestrates the long-lived `g1_slam_backend`
in process over its services and delegates single-map legs to `g1_nav`; it never
re-implements SLAM, planning, or costmap logic. A ROS-free core (version store, topology
routing, overlay, nav-point anchoring, mode/transition machine, staged editors) sits
behind thin I/O nodes.

## 🧱 What you get

- **Map lifecycle by name**: mapping a fresh map, incremental mapping on an existing
  map (relocalize then re-base then append), read-only localization, and offline
  editing. Incremental mapping materializes a new immutable version and advances the
  `active` pointer; the prior version stays intact.
- **One map-aware navigation action** (`/map_nav/navigate_to`, `g1_msgs/NavigateToAction`):
  a goal carries a map name (empty = current map). Same-map delegates a single
  `GOAL_POSE` leg to `g1_nav`; cross-map routes the captured gateway topology and switches
  maps with a controllable gateway dwell, the gateway arrival pose as the relocalization
  seed, a global acquire fallback, and a safe abort on timeout. `GOAL_NAV_POINT` is
  resolved by the manager.
- **Map-scoped nav points** (base / grasp / delivery / ...), authored live during
  mapping and anchored to the nearest keyframe so they track loop-closure
  re-optimization and stay correct across versions; read-only in localization.
- **Captured cross-map gateways**: author a gateway by standing the robot at it and
  reading the pose back from relocalization in each map (begin -> load -> commit), with a
  pre-save review of the inlier quality; no hand-typed seed. Bidirectional from one edge.
- **Non-destructive offline editing**: obstacle-mask deletion, keyframe trim + crop, and
  metadata overrides, staged with undo/redo + timestamped backups. Edits reach the
  costmap via a keyframe relay; the version snapshot and the SLAM map file are never
  modified.

## 🔌 IPC boundary (services + action + topics)

| Direction | Name (default)                          | Type                          | Notes                                                |
|-----------|-----------------------------------------|-------------------------------|------------------------------------------------------|
| action    | `/map_nav/navigate_to`                  | `g1_msgs/NavigateToAction`    | the single operator-facing nav entry                 |
| srv       | `/map_manager/start_mapping`            | `g1_msgs/StartMapping`        | fresh map by name (`/slam/reset` then MAPPING)       |
| srv       | `/map_manager/start_incremental`        | `g1_msgs/StartIncremental`    | load -> relocalize -> begin_incremental              |
| srv       | `/map_manager/start_localization`       | `g1_msgs/StartLocalization`   | load read-only (never writes the map)                |
| srv       | `/map_manager/stop_mapping`             | `g1_msgs/StopMapping`         | save the live `geometry/` (atomic swap) + lineage    |
| srv       | `/map_manager/list_maps`                | `g1_msgs/ListMaps`            | registry view                                        |
| srv       | `/map_manager/get_active_map`           | `g1_msgs/GetActiveMap`        | the current `MapManagerState`                        |
| srv       | `/map_manager/snapshot`                 | `g1_msgs/MapSnapshot`         | snapshot create/restore/list/delete                  |
| srv       | `/map_manager/add_nav_point`            | `g1_msgs/AddNavPoint`         | author (current pose or explicit); write-gated       |
| srv       | `/map_manager/query_nav_point`          | `g1_msgs/QueryNavPoint`       | resolve a point's world pose                         |
| srv       | `/map_manager/get_topology`             | `g1_msgs/GetTopology`         | maps + gateways                                      |
| srv       | `/map_manager/get_map_neighbors`        | `g1_msgs/GetMapNeighbors`     | gateways incident to a map                           |
| srv       | `/map_manager/reload_topology`          | `std_srvs/Trigger`            | re-read `topology.yaml`                              |
| srv       | `/map_manager/gateway_capture`          | `g1_msgs/GatewayCapture`      | capture a gateway: begin/load/commit/nudge/yaw/save/discard/status |
| pub       | `/map_manager/state` (latched)          | `g1_msgs/MapManagerState`     | mode, active map/version, localized, transition      |
| pub       | `/map_manager/nav_points` (latched)     | `g1_msgs/NavPointArray`       | resolved nav points                                  |
| pub       | `/map_manager/nav_point_markers`        | `visualization_msgs/MarkerArray` | RViz spheres + labels                             |
| pub       | `/map_manager/gateway_markers`          | `visualization_msgs/MarkerArray` | RViz gateway arrows + labels                      |
| sub       | `/slam/keyframe_poses`                  | `g1_msgs/KeyframePoseArray`   | full keyframe poses (nav-point anchoring)            |
| client    | `/slam/{load_map,save_map,relocalize}`  | `g1_msgs/*`                   | backend orchestration                                |
| client    | `/slam/{reset,begin_incremental}`       | `std_srvs/Trigger`            | fresh mapping / incremental re-base                  |
| client    | `/navigate_to`                          | `g1_msgs/NavigateToAction`    | single-map legs delegated to `g1_nav`                |

The editor + relay nodes add: `/map_edit/command` (`g1_msgs/MapEditCommand`) +
`/clicked_point` (`geometry_msgs/PointStamped`), and the relay `/slam/keyframes` ->
`/slam/keyframes_filtered` (`g1_msgs/KeyframeCloudArray`). TF: `map_frame` (`map`) <-
`base_frame` (`base_link`). The integration boundary is ROS topic / service / action
IPC: this package links no other package's core library.

## 🗺️ Modes

- **MAPPING**: a fresh map by name (`/slam/reset` then live keyframes).
- **INCREMENTAL**: load a map, relocalize, then `begin_incremental` re-bases and
  resumes; saving writes a new version.
- **LOCALIZATION**: load read-only; the backend hard-refuses any map write and the
  manager refuses nav-point authoring (query still allowed). Cross-map gateways are
  CAPTURED in this mode (`/map_manager/gateway_capture`: begin/load/commit/save).
- **EDITING**: offline overlay edits, non-destructive.

## 📦 First-time setup

Build the workspace (see `docs/INSTALL.md`); `g1_msgs` must be built first. At runtime
the manager needs `g1_slam_backend` (orchestrated over `/slam/*`) and, for navigation,
`g1_nav` + the planner stack. Map binaries are not in git: fetch the sample/test map
from the private HF dataset with the `g1_maps` package's `scripts/fetch_maps.py` (`hf auth login` or
`HF_TOKEN`). The python tests need `python3-nose` + `python3-yaml`.

## 🚀 Run

`map_manager_node` + the keyframe relay are always launched; upstreams are opt-in args
(default **off**), per the launch-composition convention.

```bash
# manager + relay only (the SLAM/nav stack already running separately):
roslaunch g1_map_manager map_manager.launch

# full OFFLINE stack with RViz (backend + costmap + BOTH planners + g1_nav, no robot):
roslaunch g1_map_manager map_manager.launch start_slam:=true start_costmap:=true \
    start_nav:=true start_local:=true start_global:=true rviz:=true
# then make a map active and navigate:
rosrun g1_map_manager map_cli.py start-loc demo_floor   # load a saved map read-only
# ... send a /map_nav/navigate_to goal (action client / RViz), or map a fresh one by name.

# full LIVE stack (+ frontend + sensor + robot state; e-stop in hand). start_state
# brings up g1_locomotion (the disarmed /cmd_vel bridge + real joint states), so no
# separate start_locomotion is needed. The lidar reads the robot interface from
# G1_NETWORK_INTERFACE (the launch default is $(optenv G1_NETWORK_INTERFACE)):
roslaunch g1_map_manager map_manager.launch start_slam:=true start_frontend:=true \
    start_lidar:=true start_state:=true start_costmap:=true start_nav:=true \
    start_local:=true start_global:=true rviz:=true

# offline editor:
roslaunch g1_map_manager map_edit.launch start_manager:=true rviz:=true
```

`start_nav` brings up `g1_nav`; the navigation legs it runs need the local + global
planners, so set `start_local` + `start_global` too (the costmap and SLAM backend are
owned by THIS launch and suppressed inside the `g1_nav` include to avoid double-bringup).
`g1_locomotion` is one node serving both the robot joint state and the `/cmd_vel`
bridge: `start_state` (the SLAM live chain) already brings it up, so the launch forwards
`start_locomotion` to `g1_nav` only when `start_state` is off -- it is never launched
twice. Use `start_locomotion` alone for the bridge without the SLAM state chain.

When `start_costmap` is set, the costmap's `keyframes_topic` is pointed at
`/slam/keyframes_filtered` so edits reach the static layer.

## 🕹️ Operating it

Once the stack is up, drive it with the CLI (`rosrun g1_map_manager map_cli.py ...`),
plain `rosservice call`, or the `/map_nav/navigate_to` action. See **[USAGE.md](USAGE.md)**
for the full command-by-command operating guide (one command per box, each explained):
mapping a new area, localizing and navigating, incremental extension, cross-map gateway
capture, offline editing, inspection, and the goal / timing reference.

## 🧭 How it works

A navigation goal is routed by the topology graph from the current map to the target
map. A same-map route is a single final leg sent to `g1_nav`. A cross-map route is a
sequence of legs driven by the mode machine: drive to the gateway, hold for the
effective dwell (`transition_wait`, or the per-goal `NavigateTo.transition_wait`
override), load the next map read-only, relocalize using the gateway arrival pose as
the seed (a global acquire is the fallback, a timeout is a safe abort), then send the
next leg. The execute callback owns the route state; backend calls and g1_nav legs run
off the node mutex. Incremental mapping works because `begin_incremental` re-bases each
loaded keyframe's odom pose into the live frame, so the first appended keyframe's bridge
factor is correct. Nav points store the nearest keyframe id + a relative transform and
resolve as `keyframe_world_pose * relative`, so they follow re-optimization. Edits live
in a per-map overlay; the keyframe relay drops suppressed keyframes and masked / cropped
points and republishes `/slam/keyframes_filtered`.

## ⚙️ Tuning

All knobs are flat keys in `config/map_manager.yaml`, with a code default the YAML
overrides: `transition_wait` (gateway dwell), `reloc_seed_timeout` /
`reloc_global_timeout` (relocalization budgets), `transition_timeout`, `state_rate`,
`nearest_anchor_max_dist` (nav-point anchoring), and `delete_radius` (obstacle-mask
deletion). Action / topic / service names are params too (remappable in the launch). No
value is hardcoded in the nodes.

## 🧪 Build & test

```bash
# from catkin_ws/:
catkin build g1_map_manager && source devel/setup.bash
catkin run_tests g1_map_manager --no-deps && catkin_test_results build/g1_map_manager
```

The ROS-free cores are covered by C++ gtests (version store, topology routing, overlay,
nav-point anchoring, mode machine, the staged editors, the keyframe filter). A python
launch/param contract test guards the launch composition and the YAML/param contract. An
end-to-end `rostest` brings up `map_manager_node` against a single mock node that plays
the SLAM backend + `g1_nav`, exercising the lifecycle, nav-point anchoring + tracking,
the read-only proof, same-map + cross-map navigation (seed-hit / global-fallback /
abort), gateway authoring, and content editing:

```bash
rostest g1_map_manager map_manager_integration.test
```

The remaining acceptance is the owner-in-the-loop live gate (non-blocking, e-stop in
hand): map a small area by name and author nav points; localize read-only and confirm
files are unchanged, then navigate to a point; author a gateway and run a cross-map goal,
watching the seed-hit switch, a forced seed miss falling back to a global acquire, and an
unreachable seed aborting safely; incrementally extend a map and confirm a new version
with the prior intact.
