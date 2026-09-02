# g1_costmap - agent guide

2D layered occupancy costmap, ground-relative 3-state, probabilistic (log-odds)
fusion with raycast free space and a normal-gated ground/obstacle split.
In: /slam/keyframes (g1_msgs/KeyframeCloudArray: per-keyframe map cloud + sensor
origin, from the backend), /slam/frontend/cloud_registered (live PointCloud2,
odom frame), TF (map<-odom, map<-mid360_link, map<-*_ankle_roll_link,
map<-base_link). Out:
/nav/costmap (nav_msgs/OccupancyGrid, INFLATED: free(0)/obstacle(100)/unknown(-1)
+ inflation band(1..99); for the planner), /nav/costmap_raw (OccupancyGrid, 3-state
NO inflation: free(0)/obstacle(100)/unknown(-1); the pre-inflation 3-state grid map via
rviz/Map "map" scheme = white/black/gray, the DEFAULT RViz display),
/nav/costmap_prob (OccupancyGrid: static log-odds probability 0..100, no inflation;
off-by-default diagnostic), /nav/change_prob (OccupancyGrid: the dynamic CHANGE
layer's occupancy probability 0..100, -1 neutral, no inflation; off-by-default
diagnostic), and /nav/local_costmap (OccupancyGrid: a rolling
robot-centered crop of the inflated master, 6 m x 6 m default @ costmap resolution,
map frame, for the MPC local planner). The inflation view is
/nav/costmap with the standard rviz/Map "costmap" scheme (no custom RGB cloud). Own
C++ reimplementation; the ground/band/classifier follows
log-odds accumulation + normal-angle ground segmentation + erosion are
implemented in standalone C++.

## Boundary (do not break)
- Integration boundary is ROS-topic IPC: other packages consume /nav/costmap and
  /nav/local_costmap; do not export a linkable costmap library. This node owns NO TF
  besides the reads it needs for its own inputs (feet + lidar + base_link).
- This node produces the GLOBAL grid and the rolling local window
  /nav/local_costmap (a robot-centered crop of the inflated master, NOT a second
  fusion path); do not add further speculative windows.

## Layout
Headers under `include/g1_costmap/{core,layers}/`, sources under `src/{core,layers}/`
(parallel); the node stays at `src/` root. Include paths are `g1_costmap/core/...`
and `g1_costmap/layers/...`. All ROS-free, std::thread-free, unit-testable, built as
lib `g1_costmap_core`.
- `core/` (data model + pure primitives): `cost_values.h` (cost constants +
  costToOccupancy), `ground_classifier.h` (layerOf / classify3 / classify3WithNormal
  / normalAngleFromUp / bandTopLayers, pure), `foot_ground.h` (footGroundFromContacts:
  kinematic floor from the sole spheres, pure), `costmap_grid.{h,cpp}`,
  `ground_surface.{h,cpp}` (per-cell lowest ground + fill + GroundLookup),
  `occupancy_accumulator.{h,cpp}` (log-odds Bayesian fusion),
  `local_window.{h,cpp}` (cropCentered: robot-centered crop of the inflated master
  for the local MPC window, ROS-free).
- `layers/`: `static_layer.{h,cpp}` (LocalGridMaker + accumulate + erodeObstacleSpeckle),
  `dynamic_layer.{h,cpp}`, `inflation_layer.{h,cpp}`, `layered_costmap.{h,cpp}`
  (config + owner + fuse).
- Node (ROS I/O only): `src/costmap_node.cpp` -> `g1_costmap_node`. PCL normal
  estimation lives only in the node (keyframe clouds).

## Invariants (do not break)
- **Obstacle domain = ground -> LIDAR PLANE (not a fixed height).** Per point:
  `offset = layerOf(z) - layerOf(ground_z(cell))` at the cell pitch;
  `offset <= walkable_layers` (at/below ground) -> FREE; above walkable up to the
  band TOP -> OBSTACLE; above the band top -> IGNORED (overhead the robot walks
  under). The band TOP is the LIDAR PLANE: `bandTopLayers` = `layerOf(lidar_z -
  phantom_drop) - layerOf(ground_z)`, where lidar_z is the keyframe origin z
  (static) or the `mid360_link` z (dynamic), capped by a generous fixed safety
  bound `obstacle_max_layers = round(obstacle_max_height / resolution)` (default
  2.0 m, normally NOT the binding cap) and falling back to it if lidar_z degenerates
  to at/below ground. Returns above the lidar plane are ignored (ceiling domain),
  making a tall sub-lidar table fully obstacle. Below ground is FREE (owner model). Do NOT
  reintroduce a fixed static_z_min/max or a ground-relative-only band top.
- **Ground = per-cell lowest, foot-contact fallback.** `GroundSurface` stores the
  lowest z per cell (the floor is the lowest return), trusted after
  `ground_min_points`, neighbour-filled within `max_fill_radius`; `GroundLookup`
  falls back to the foot scalar. The node computes that scalar in `updateFootGround`
  via `footGroundFromContacts`: it projects the foot sole collision-sphere centres
  (the `foot_*` params, from g1_description ankle_roll) through BOTH
  `*_ankle_roll_link` TFs and takes the min map Z minus the sphere radius, so it is
  exact under foot tilt and the min over points picks the stance (lower) foot (set
  via setFootGround). `base_footprint` is NOT the floor (it projects to odom z=0 =
  pelvis-start height ~0.7 m up); do not use it as ground. A point with neither
  surface ground nor a foot reading is skipped (GroundLookup::valid == false).
- **Static layer = per-keyframe LOCAL GRID -> log-odds GLOBAL MAP.** Two-stage
  (local grid per keyframe -> shared log-odds accumulator). Per keyframe (each: sensor origin +
  map-frame cloud, from g1_slam_backend's publishOutputs) the LocalGridMaker
  segments returns (band refined by the normal gate), then in TWO passes: (1) mark
  every OBSTACLE cell, (2) raycast origin -> each return clearing the beam FREE but
  STOPPING at the first obstacle cell, and angular-fills the unobserved wedges (also
  stopping at obstacles). The stop-at-obstacle is the OCCLUSION guard: a beam that
  in 3D passes over an obstacle must not clear the obstacle's 2D footprint or the
  cells occluded behind it (else a tall table's occluded centre reads FREE). The
  occluded interior stays UNKNOWN (stop-at-obstacle occlusion guard; the node
  never invents 2D free over an obstacle). It casts ONE hit/miss vote
  per touched cell into a shared OccupancyAccumulator (log-odds). The accumulator
  is thresholded to 3-state, isolated obstacle speckle is eroded, and residual
  holes are morphologically closed. This replaced last-write-wins: a cell needs
  consistent evidence to read OBSTACLE and free observations vote noise/drift
  obstacles back out. Grid origin/size from robust XY percentile bounds + margin;
  the dynamic layer adopts that geometry; rebuilt on each full-set latched keyframe
  message (loop-closure re-optimization reflected). The accumulator also yields the
  per-cell probability published on /nav/costmap_prob (pre-erode/close, raw fusion
  confidence). `setStaticCloud` (bare last-write projection) is kept for tests only.
  Do NOT revert the static source to /slam/map: the assembled cloud has no
  per-point origin, so no raycast, and the voxel map is too sparse for normals.

- **Normal gate is a REFINEMENT of the height band, not a replacement.** Per point
  `classify3WithNormal` runs `classify3` first, then: a near-horizontal surface
  (normal within `ground_normal_angle` of vertical) up to `walkable_layers +
  normal_flat_grace_layers` above ground demotes OBSTACLE->FREE (drifted/uneven
  floor); a near-vertical surface (normal at least `vertical_normal_angle` off
  vertical) at/above the ground layer promotes FREE->OBSTACLE (wall base).
  Below-ground and IGNORED (ceiling) are never reclassified. Normals are estimated
  per keyframe on the dense map-frame cloud (KNN PCA, `normal_k`); a keyframe with
  no/empty normals (or `use_normals:=false`) falls back to pure height bands. The
  angle is `|nz|`-based, so normal orientation does not matter.
- **3-state semantics.** Cells default UNKNOWN (NO_INFORMATION=255). FREE=0,
  OBSTACLE=LETHAL_OBSTACLE=254, inflation in (0,253], INSCRIBED=253. The node maps
  to OccupancyGrid via costToOccupancy (255->-1, 254->100 lethal, 253->99 inscribed,
  0->0, band->1..98) -- inscribed(99) is DISTINCT from lethal(100) so the global
  planner can relax inscribed near a goal and the local footprint filter checks the
  true body vs true-lethal only.
  Fusion is three-way asymmetric on the dynamic change log-odds vs the static cell:
  log-odds >= logodds(dyn_add_thr) -> LETHAL (add obstacle, any static); else
  log-odds <= -logodds(dyn_clear_thr) AND static LETHAL -> FREE (clear a mapped
  obstacle, e.g. an opened door; HIGH bar); else log-odds <= -logodds(dyn_fill_thr)
  AND static UNKNOWN -> FREE (fill unknown; low bar); else defer to static. Then
  inflate.
  Inflation's `cost > current` guard leaves UNKNOWN (255) and LETHAL (254) untouched
  (no inflation cost exceeds 254), so unknown space stays unknown - keep that guard.
- **Dynamic layer = bidirectional log-odds CHANGE evidence; never writes the SLAM
  map.** Source is cloud_registered (odom frame), transformed to map at the cloud
  stamp, classified on the SAME GroundLookup (ground->lidar-plane band). A raycast
  votes a MISS (lowers per-cell log-odds) on traversed cells with the SAME two-pass
  stop-at-obstacle occlusion guard (per-update obs_mask_; a beam over a current
  obstacle does not clear behind it), but it DOES pass through a static-LETHAL cell
  that has no live return (an opened door) and votes it free. An obstacle return
  votes a HIT (raises log-odds). A cell is only voted an OBSTACLE when a supported
  FRACTION of its returns are in-band (dyn_obstacle_frac) with at least
  dyn_min_returns returns; a sparse in-band cell is left neutral (the temporal
  log-odds decides). This is the grid-native RTAB-Map cluster-min-size analog that
  suppresses the false ground ring from live-cloud Z scatter, without raising
  walkable_layers (which would blind the planners to low obstacles). Every tick all
  non-neutral cells relax toward 0 by
  dyn_decay_half_life, so an unobserved obstacle ghost fades while a re-observed cell
  holds (one vote per cell per update). Sensor origin for raycast AND the
  lidar-plane band top = map<-mid360_link. The static grid DATA and map FILE are
  never modified -- clearing shows up only in the fused master.
- **No self-filter (owner-confirmed from the FOV).** The inverted Mid-360 does not
  see the legs/feet (only the arms, within the frontend 0.5 m blind), so
  cloud_registered has no self-returns. Do NOT add a self-filter or footprint-clear
  unless a mount/FOV change surfaces self-returns.
- **Params in the node's PRIVATE namespace**, FLAT keys from config/costmap.yaml so
  dynamic_reconfigure reads the live subset directly. Every param has a code default
  the YAML overrides. Live knobs via cfg/CostmapTuning.cfg ->
  g1_costmap::CostmapTuningConfig (named to avoid clashing with the core
  g1_costmap::CostmapConfig struct). setConfig stores only; the GroundSurface is
  (re)built in setStaticCloud, so a live reconfigure never wipes it.
- **Threading.** Core is std::thread-free. The node runs AsyncSpinner(2) with one
  mutex guarding the cloud cache + costmap_. The timer copies the cloud ptr under
  the lock, does TF lookups + the point transform OFF the lock, then updateDynamic +
  fuse + build-message under the lock. The last-good TF caches (cloud/base/feet) are
  touched only by the timer thread (lookup runs on the timer), so they need no extra
  lock. TF miss reuses the last good transform (throttled warning).

## clear_costmap service
`g1_costmap_node` advertises `clear_costmap` (`std_srvs/Empty`) in its private
namespace. Calling it resets the dynamic (transient-obstacle) layer to all
NO_INFORMATION: stale obstacles are flushed, the static SLAM map is untouched,
and the next timer tick re-fuses and republishes the cleared master. A
still-present obstacle reappears on the very next dynamic update -- this clears
stale/transient marks, not permanent walls. Gated by `clear_service_enable`
(default true); the service name is `clear_service_name` (default "clear_costmap"),
both flat keys in `config/costmap.yaml`. Called by `g1_nav` recovery as the first
cheap recovery step; a cooldown on the caller's side guards against thrashing.

Clearing now also zeroes the dynamic change log-odds (DynamicLayer::clear),
so a wrongly-cleared static obstacle (a false door-open) reverts to LETHAL until
re-observed past dyn_clear_thr. The change layer's dyn_decay_half_life self-heals
transient ghosts before recovery fires, so this service is a backstop.

## Localization vs mapping

The durable "door is open" memory belongs to the MAPPING phase: the static keyframe
log-odds (prob_hit/prob_miss/clamp) bakes the door's latest persistent state into the
SLAM map. In LOCALIZATION the map is frozen; set freeze_static:=true so the node
ignores keyframe updates after the first static build, and the change layer handles
live avoidance + door clearing purely in memory, never writing the map file.

## Build / test
```bash
# from catkin_ws/:
catkin build g1_costmap && source devel/setup.bash
catkin run_tests g1_costmap --no-deps && catkin_test_results build/g1_costmap
```
C++ gtests (classifier, ground surface, grid, static/dynamic ground-relative
classification, inflation, fusion) + a python launch/config contract check. The
full estimator is gated by an offline replay (saved map + cloud_registered bag
with the leg TF) + RViz with a moved synthetic obstacle. Node binary
g1_costmap_node; node name g1_costmap.

See docs/INSTALL.md for build and environment setup.
