# g1_map_manager - agent guide

Map management + map-aware navigation for the G1. THIN ROS nodes over a ROS-free
core. Orchestrates the long-lived g1_slam_backend in process via services, owns the
single operator-facing map-aware navigation entry, and delegates single-map legs to
the existing g1_nav action. Do NOT re-implement SLAM, planning, or costmap logic here.

## Status
Core (ROS-free, gtest-covered): pose_math, map_lineage, map_store, topology_graph (+
topology YAML IO), overlay, nav_point_store (undo/redo), mode_machine, gateway_editor,
gateway_capture, map_editor. Nodes: map_manager_node (lifecycle + state +
/map_nav/navigate_to action + backend orchestration + nav points + snapshot/restore +
topology queries + markers + gateway capture),
map_edit_node, keyframe_overlay_node. CLI: scripts/map_cli.py. Tests: core gtests +
a python launch/param contract test + a mock-backend/mock-g1_nav integration rostest
(lifecycle, nav-point anchoring+tracking, read-only proof, same-map + cross-map nav,
gateway capture, content editing). Owner-in-the-loop (non-blocking, e-stop in hand):
live map by name + author nav points; localize read-only + navigate to a point;
capture a gateway (begin/load/commit/save) and run a cross-map goal (seed-hit, forced
seed-miss fallback, unreachable abort); incrementally extend a map.

## Boundary (do not break)
- ROS-free core: include/g1_map_manager/core/ and src/core/ contain no ROS headers
  and no std::thread. In-memory poses are Eigen::Isometry3d; Pose3 is ONLY the
  YAML/msg DTO. The nodes are the sole ROS surface (mirror g1_nav). keyframe_filter.h
  is node-adjacent (uses the g1_msgs cloud type + PCL), header-only + gtested.
- Integration is ROS topic/service/action IPC: the manager talks to the backend via
  /slam/* services and to g1_nav via the /navigate_to action; it links no other
  package's core library.
- LOCALIZATION never writes a map file: the backend read-only guard is the hard
  enforcement; the manager also gates nav-point writes on the mode machine
  (canWriteMap()). Offline overlay edits are non-destructive (overlays/*.yaml, the
  geometry is never touched).
- Layout = geometry/ + overlays/ + snapshots/ per map (see g1_maps CONTRACT.md).
  geometry/manifest.yaml is BACKEND-owned (never written by the manager);
  geometry/lineage.yaml is the manager's version sidecar. Saving has the backend write
  geometry.tmp/ which the manager commits by an atomic dir swap; incremental
  auto-stamps a pre-incremental snapshot first (the rollback point). Mutable overlay +
  topology writes go through a bounded .history/ (writeFileWithHistory); rollback is
  restoreSnapshot. There is no versions/ tree or active pointer.
- Threading: AsyncSpinner + one mutex mu_. NEVER call a blocking backend service
  .call() (or wait on a g1_nav leg) while holding mu_ -- gather under the lock,
  release, do the ROS I/O, re-acquire to commit. The /map_nav/navigate_to execute
  callback owns the cross-map route state (actionlib serializes goals).
- No hardcoded values: every path/threshold/topic/service/action name from
  config/map_manager.yaml (code defaults overridden by YAML).

## Layout
- core/map_types.h            -- Mode, Pose3 (DTO), NavPointRec, GatewayEdge,
                                 KeyframePoseRec; toIso/fromIso
- core/map_lineage            -- MapLineage DTO (geometry/lineage.yaml) + read-only
                                 backend-manifest num_keyframes parse
- core/map_store              -- geometry/overlays/snapshots layout, atomic geometry
                                 commit, snapshot create/restore/list/prune,
                                 writeFileWithHistory (fs injected via MapFs)
- core/topology_graph         -- Dijkstra routing + topology.yaml parse/serialize
- core/overlay                -- obstacle/trim/metadata model + insidePolygon
- core/nav_point_store        -- keyframe-anchored nav points + YAML IO
- core/mode_machine           -- mode + cross-map transition (gateway dwell)
- core/gateway_editor         -- staged gateway edge (undo/redo, dedup-replace)
- core/gateway_capture        -- begin/load/commit capture FSM (ROS-free)
- core/map_editor             -- staged overlay content editing (undo/redo)
- keyframe_filter.h           -- apply an overlay to a KeyframeCloudArray (node-adjacent)
- node_fs.h                   -- node-only: build a MapFs from std::filesystem (shared
                                 by map_manager_node + map_edit_node; not in the core)
- src/map_manager_node.cpp    -- the thin orchestrator (lifecycle, state, action,
                                 queries, snapshot/restore, gateway capture + topology write)
- src/map_edit_node.cpp       -- MapEditCommand editing + /clicked_point picking
- src/keyframe_overlay_node.cpp -- /slam/keyframes -> /slam/keyframes_filtered
- scripts/map_cli.py          -- thin CLI over the manager services
- config/map_manager.yaml     -- flat params; launch/{map_manager,map_edit}.launch
- config/map_manager.rviz     -- costmap + nav-point markers + gateway markers + tools

## Key decisions
- Eigen::Isometry3d for all in-memory SE(3) math (compose = a * b, inverse =
  .inverse()); Pose3 only crosses the YAML/msg boundary.
- The manager orchestrates the backend in process; incremental = load -> relocalize
  -> begin_incremental (re-base). Cross-map = per-leg scheduling with a controllable
  gateway dwell before the switch, the gateway arrival pose as the relocalization
  seed, a global acquire as the fallback, and a safe abort on timeout. Same-map is a
  single final leg through the same machine.
- Goals carry a map name; an empty name means the current map. GOAL_NAV_POINT is
  resolved by the manager (never the backend) from the active map's nav store + the
  live keyframe poses, so the backend stays a pure SLAM service.
- Edit overlays are applied by the manager-side keyframe relay (not the backend), so
  the backend and the /slam/keyframes contract stay untouched; the relay output feeds
  the costmap via its keyframes_topic param.
- Editors are staged (define/edit -> undo/redo -> save); on save the node routes the
  overlay file through a bounded overlays/.history/ (writeFileWithHistory). The ROS-free
  editor cores hold the logic, the nodes add TF/clicked-point capture + I/O.
- Snapshots: rare full restore points (geometry/ + overlays/). The manager auto-stamps a
  <ts>-pre-incremental snapshot before each increment (rollback) and the operator can
  stamp milestones via /map_manager/snapshot (op create|restore|list|delete); restore is
  a copy-back. There is no version pinning.
- Gateways are CAPTURED during LOCALIZATION, not hand-typed: begin (from-side pose from
  the map->base_link TF) -> load (operator-timed target load) -> commit (global
  relocalize, read the locked pose + RelocQuality) -> review/nudge -> save; the manager
  owns the topology/topology.yaml write (via bounded topology/.history/) + reload.
  /slam/relocalize carries RelocQuality for the accept gate + review.

## Build / test
```bash
# from catkin_ws/:
catkin build g1_map_manager && source devel/setup.bash
catkin run_tests g1_map_manager --no-deps && catkin_test_results build/g1_map_manager
```
NOTE: `catkin run_tests` does not rebuild the node executables; run `catkin build`
first when a node or launch changed, else the rostest runs a stale binary.
