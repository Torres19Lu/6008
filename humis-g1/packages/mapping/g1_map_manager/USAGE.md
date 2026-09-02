# g1_map_manager operating guide 🕹️

Command-by-command guide for driving a running `g1_map_manager` stack. Each box is a
single command with a one-line note, so each feature can be read on its own. For
bringing the stack up (launch args, offline vs live), see the **🚀 Run** section of
`README.md`; for what each mode does, see **🗺️ Modes**.

## 🧰 Three ways to drive it

You can issue every operation in one of three ways. Pick whichever fits the moment.

The CLI wrapper, best for routine operations:

```bash
rosrun g1_map_manager map_cli.py status
```

A raw service call, for full control of every field:

```bash
rosservice call /map_manager/get_active_map "{}"
```

The navigation action, for goals (here via the actionlib GUI):

```bash
rosrun actionlib axclient.py /map_nav/navigate_to
```

## 🆕 Map a new area

Enter MAPPING on a fresh map by name (runs `/slam/reset`, then builds live in RViz):

```bash
rosrun g1_map_manager map_cli.py start-mapping floor1
```

Teleop or walk the robot to cover the area, then author a nav point at the CURRENT pose
(anchored to the nearest keyframe):

```bash
rosrun g1_map_manager map_cli.py add-point base
```

Add as many named points as you need (base / grasp / delivery / ...):

```bash
rosrun g1_map_manager map_cli.py add-point grasp_point
```

Author a point at an EXPLICIT pose instead of the current one:

```bash
rosservice call /map_manager/add_nav_point "{name: dock, use_current_pose: false, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}, source: 1}"
```

Save the result (the backend writes a fresh `geometry/`, committed by an atomic swap):

```bash
rosrun g1_map_manager map_cli.py stop --save
```

Or stop WITHOUT saving to discard the run (omit `--save`):

```bash
rosrun g1_map_manager map_cli.py stop
```

## 📍 Localize on a saved map and navigate

Load a saved map read-only into LOCALIZATION (never writes the map file):

```bash
rosrun g1_map_manager map_cli.py start-loc floor1
```

Check the current mode / active map + label / localized / transition:

```bash
rosrun g1_map_manager map_cli.py status
```

Navigate to a nav point in the current map (`goal_type: 2` = `GOAL_NAV_POINT`):

```bash
rostopic pub -1 /map_nav/navigate_to/goal g1_msgs/NavigateToActionGoal '{goal: {goal_type: 2, nav_point_name: "grasp_point", map_name: "", transition_wait: -1.0}}'
```

Navigate to a pose in the current map's `map` frame (`goal_type: 0` = `GOAL_POSE`):

```bash
rostopic pub -1 /map_nav/navigate_to/goal g1_msgs/NavigateToActionGoal '{goal: {goal_type: 0, map_name: "", target_pose: {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}}'
```

A same-map pose goal (empty `map_name`) can also come straight from the RViz **2D Nav
Goal** tool, which reaches `g1_nav` directly. The GUI alternative to `rostopic pub`:

```bash
rosrun actionlib axclient.py /map_nav/navigate_to
```

See **📖 Goal and timing reference** below for `goal_type`, `map_name`, and
`transition_wait`.

## ➕ Incrementally extend a map

Resume an existing map in INCREMENTAL (load -> relocalize -> re-base -> resume):

```bash
rosrun g1_map_manager map_cli.py start-inc floor1
```

Walk the new area, then save (the manager auto-stamped a `pre-incremental` snapshot
before the run as a rollback point; the save swaps in the new `geometry/`):

```bash
rosrun g1_map_manager map_cli.py stop --save
```

## 📸 Snapshots and rollback

List a map's snapshots (the auto `pre-incremental` points plus any milestones):

```bash
rosrun g1_map_manager map_cli.py snapshot list --name floor1
```

Stamp a named milestone snapshot of the current map:

```bash
rosrun g1_map_manager map_cli.py snapshot create --name floor1 --label "before cleanup"
```

Roll the live map back to a snapshot (restores `geometry/` + `overlays/`):

```bash
rosrun g1_map_manager map_cli.py snapshot restore --name floor1 --id 20260624-093000-pre-incremental
```

Read the readable lineage (current + snapshots, with keyframe counts and parents):

```bash
rosrun g1_map_manager map_cli.py history floor1
```

## 🌉 Capture a cross-map gateway and navigate across maps

A gateway is an edge between two maps, captured by reading the robot's pose back from
relocalization on each side. One edge is bidirectional.

Localize on the from-side map and stand the robot at the gateway point:

```bash
rosrun g1_map_manager map_cli.py start-loc floor1
```

Begin the capture (records the from-side pose in `floor1`; `--label` names the edge):

```bash
rosrun g1_map_manager map_cli.py gateway-begin floor2 --label stairs
```

Drive the robot to the to-side (ride the elevator for a non-overlapping gateway), then
load the to-side map read-only:

```bash
rosrun g1_map_manager map_cli.py gateway-load
```

Commit (global relocalize on the to-side, stage the captured edge + inlier ratio):

```bash
rosrun g1_map_manager map_cli.py gateway-commit
```

Optionally nudge the staged seed in metres before saving:

```bash
rosrun g1_map_manager map_cli.py gateway-nudge 0.10 -0.05
```

Optionally rotate the staged seed yaw in radians:

```bash
rosrun g1_map_manager map_cli.py gateway-yaw 0.05
```

Review the staged arrow + inlier ratio in RViz, then save (writes
`topology/topology.yaml`, prior versions rolled into `topology/.history/`, and reloads):

```bash
rosrun g1_map_manager map_cli.py gateway-save
```

Confirm the new edge:

```bash
rosservice call /map_manager/get_map_neighbors "{map_name: floor1}"
```

A goal in the other map now routes across the gateway, holding 5 s at it before the
switch:

```bash
rostopic pub -1 /map_nav/navigate_to/goal g1_msgs/NavigateToActionGoal '{goal: {goal_type: 2, nav_point_name: "delivery", map_name: "floor2", transition_wait: 5.0}}'
```

For an OVERLAPPING gateway (the robot does not move), one command chains
begin -> load -> commit, then prompts you to save or discard:

```bash
rosrun g1_map_manager map_cli.py gateway-capture floor2 --label doorway
```

Discard a staged capture without writing:

```bash
rosrun g1_map_manager map_cli.py gateway-discard
```

## ✏️ Edit a map offline (non-destructive)

Edits live in a per-map overlay; the `geometry/` and the SLAM map file are never
modified, and the result reaches the costmap via `/slam/keyframes_filtered`.

Bring up the editor (with the manager and RViz):

```bash
roslaunch g1_map_manager map_edit.launch start_manager:=true rviz:=true
```

Pick the map to edit (LOCALIZATION load):

```bash
rosrun g1_map_manager map_cli.py start-loc floor1
```

In RViz, use **Publish Point** on stale obstacles; each click publishes a
`/clicked_point`, and the editor deletes points within `delete_radius` of it.

Trim a whole keyframe out of the map:

```bash
rosservice call /map_edit/command "{command: trim_keyframe, keyframe_id: 42}"
```

Override a metadata field:

```bash
rosservice call /map_edit/command "{command: set_metadata, key: description, value: north wing}"
```

Undo the last staged edit:

```bash
rosrun g1_map_manager map_cli.py undo
```

Redo it:

```bash
rosrun g1_map_manager map_cli.py redo
```

Save all staged edits (writes `overlays/*.yaml`; the prior file rolls into `overlays/.history/`):

```bash
rosservice call /map_edit/command "{command: save}"
```

Discard all staged edits instead:

```bash
rosrun g1_map_manager map_cli.py discard
```

## 🔎 Inspect anytime

List every map with its current label, keyframe count, and snapshot count:

```bash
rosrun g1_map_manager map_cli.py list
```

Show the current state (mode / active map + label / localized / transition):

```bash
rosrun g1_map_manager map_cli.py status
```

Dump the full cross-map topology (maps + gateways):

```bash
rosservice call /map_manager/get_topology "{}"
```

Resolve a nav point's world pose:

```bash
rosservice call /map_manager/query_nav_point "{query: base}"
```

Read one latched state message:

```bash
rostopic echo -n1 /map_manager/state
```

## 📖 Goal and timing reference

Fields on a `/map_nav/navigate_to` goal (`g1_msgs/NavigateToAction`):

| Field             | Values / meaning                                                        |
|-------------------|-------------------------------------------------------------------------|
| `goal_type`       | `0` = `GOAL_POSE` (use `target_pose`), `2` = `GOAL_NAV_POINT` (use `nav_point_name`) |
| `map_name`        | empty = the current map; a name routes (same-map = one leg, cross-map = over the gateway graph) |
| `transition_wait` | `-1` = the configured gateway dwell, `>= 0` overrides it (`0` = no dwell) |

`transition_wait` and the relocalization budgets are tuned in `config/map_manager.yaml`
(see the **⚙️ Tuning** section of `README.md`).
