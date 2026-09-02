# g1_slam_backend - agent guide

SLAM back end: keyframe pose graph with Scan Context loop closure + GTSAM iSAM2.
In: /slam/frontend/odom + /slam/frontend/cloud_registered. Out: TF map->odom,
/slam/map (latched), /slam/pose_graph, /slam/keyframes (latched),
/slam/keyframe_poses (latched; full poses, no clouds, for nav-point anchoring).
Services: /slam/save_map, /slam/load_map (g1_msgs; LoadMap.read_only freezes the map
AND refuses disk writes), /slam/relocalize (g1_msgs; returns locked_pose on success),
/slam/reset + /slam/begin_incremental (std_srvs/Trigger; reset to fresh mapping /
resume mapping on a loaded map after a lock). Reimplemented (not ported) from the
published Scan Context loop detection + iSAM2 (GTSAM) optimization; front end is a
black box.

## Boundary (do not break)
- Integration boundary is ROS-topic IPC: other packages consume topics only; do
  not export a linkable library (IPC via ROS topics only). The frontend stays an untouched
  black box - the backend DERIVES keyframes from the frontend's odom+cloud; do
  NOT add a /slam/keyframe publisher to the frontend or make it depend on g1_msgs.
- This node owns map->odom ONLY and never publishes anything below odom. No
  front-end feedback: odom stays smooth, map->odom absorbs loop jumps (REP-105).

## Layout
- Core (ROS-free, std::thread-free, unit-testable): `pose_graph.{h,cpp}` (iSAM2,
  Eigen-only pimpl), `scan_context.{h,cpp}` (descriptor + ring-key DB + gravity
  align), `backend.{h,cpp}` (orchestration), `tracking_correction.{h,cpp}`
  (tracking-correction projection/clamp/low-pass/jump-reject; pure SE(3) math),
  `keyframe.h`. Built as lib `g1_slam_backend_core`.
- Node (ROS I/O only): `backend_node.cpp` -> `g1_slam_backend_node`.

## Invariants (do not break)
- **Storage 3 (base_link-local clouds).** Keyframe cloud = `P_i^-1 * cloud_world`
  (base_link frame) paired with `P_i = odom->base_link`. Map = union of
  `X_i * C_base_i`; correction `map->odom = X_n * P_n^-1`. Do NOT store world-frame
  (large-coord float32 erosion) or IMU-body clouds (needs the dynamic waist
  extrinsic). The waist is already baked into `P_i`, so the backend never touches
  waist_yaw or sensor-frame TF.
- **Scan Context frame = yaw-only gravity-aligned.** Build the descriptor on
  `toScanContextFrame(cloud_base, odom_R_base)` (strips gait roll/pitch, keeps
  heading); SC bins by height, so the height axis must be gravity-aligned. CAVEAT:
  the SC frame is base_link-CENTERED (origin at the pelvis), not ground-centered,
  so `sc/height_offset` (default 2.0) must exceed the largest structure depth
  BELOW the base origin or those bins go negative and collide with the "0 == empty"
  cosine invariant. Fine for normal indoor floors (~0.7 m below base); validate it
  if operating near deep drops (stairwells, mezzanine edges) more than ~2 m down.
- **map->odom TF is now()-stamped** (+ transform_tolerance), re-broadcast at a
  fixed rate, value updated on optimization. Reusing the optimization stamp would
  trip TF_REPEATED_DATA (the g1_description base_footprint lesson). A
  non-advancing-stamp guard also skips duplicate stamps under sim-time bag replay
  (now() advances coarsely with --clock); it is a no-op on wall clock.
- **Threading.** `backend.h` is std::thread-free (pure logic). The node runs the
  fast `stepIntake` on the odom+cloud sync and the slow `runLoopClosureOnce` on a
  separate timer, serialized by one mutex (AsyncSpinner(2)). Keep loop detection
  off the intake path. Map publishing also stays off the mutex: the node takes a
  cheap `snapshotKeyframes()` under the lock, then runs the heavy
  `assembleFromSnapshot()` (transform + concat + voxel) lock-free, so the dense
  (0.1/0.1) map does not stall intake.
- **Params in the node's PRIVATE namespace**, loaded from
  config/slam_backend.yaml via `<rosparam>` inside `<node>`. No hardcoded
  values; every param has a code default the YAML overrides.
- **GTSAM** is a system dep via `find_package(GTSAM)` (ROS GTSAM 4.2.0); no rosdep
  tag. Pin one install; an ABI/Eigen mismatch shows as a runtime segfault, which
  `test/test_gtsam_abi.cpp` guards against.
- **Persistence + localization.** Map dir = `manifest.yaml` +
  `pose_graph.g2o` (own minimal SE3:QUAT text IO, GTSAM-version-independent;
  vertices=optimized poses, edges=odom (`from==to-1`) + loop) + `scan_context.bin`
  (f64 descriptors; ring keys recomputed on load) + `keyframes/NNNNNN.pcd`
  (`base_link` clouds). `saveMap` is atomic (sibling temp dir + swap). `loadMap`
  adopts the map's SC config, rebuilds iSAM2, and leaves `map->odom` UNSET
  (Identity): relocalization establishes it, not load. Loop-edge ICP measurements
  are retained (`Backend::LoopEdge.rel`) so g2o is faithful + re-optimizable. After
  load the map is FROZEN (localization-only): no new keyframes, no loop closure, and
  `map->odom` is NOT broadcast until the first lock. NOTE: with the Cauchy kernel on
  (`loop/robust_c > 0`) the optimization is NON-CONVEX, so a batch reload re-optimizes
  to slightly different poses than the incremental live build (g2o MEASUREMENTS are
  faithful; the optimized vertices are not bit-identical). The map_io round-trip test
  therefore sets `loop_robust_c = 0` (plain = convex = exact) to test serialization.
- **Loop-closure robustness (do not drop).** SC + ICP-fitness alone produces FALSE
  loops in corridors (SC is translation-invariant along the axis and yaw-invariant
  via column shift, so opposite-heading passes match) and repetitive rooms; one bad
  loop in a plain-Gaussian graph distorts the WHOLE map (the no-loop regions stay
  perfect, the looped regions twist). Two layers, both in `runLoopClosureOnce` /
  `addLoop`: (1) a **front-end consistency gate** rejects a loop whose ICP rel
  disagrees with the odometry-chained relative `P_cand^-1 * P_id` beyond
  `loop/max_dt + loop/max_dt_rate * path` (or the `max_dr` rotation counterpart) -
  valid because the front end is accurate, so a large disagreement is a false/
  degenerate match, not drift; (2) survivors are added under a **Cauchy robust
  kernel** (`pose_graph.cpp::loopNoise`, `loop/robust_c`) that down-weights an
  outlier by its residual (Cauchy M-estimator on each loop factor). Loop sigmas are LOOSER
  than odom on purpose (trust = kernel, not sigma). Validated offline on a real map
  (`tmp/g1_bigmap`): plain Gaussian 4.34 m RMS distortion vs gate+Cauchy 0.53 m.
  Two supporting choices: iSAM2 is built with `relinearizeSkip=1` (`pose_graph.cpp::
  isamParams`), NOT the GTSAM default 10, so a loop correction relinearizes every
  update and propagates immediately; and the loop ICP
  TARGET submap (only) is re-voxeled at `icp/loop_voxel` before ICP so point-to-
  point ICP is not biased by uneven keyframe overlap. Do NOT voxel the single-
  keyframe SOURCE (`kf.cloud_base`): it is already uniform at keyframe_voxel, and
  re-voxeling a single scan to a coarser leaf only thins it and starves ICP of
  correspondences on drifted loops ("Not enough correspondences"). The loop + reloc
  ICP TARGET submap is `icp/submap_neighbors` = 8 (raised from 2 after the live gate:
  on a real map a +/-8 submap roughly halved loop fitness and tightened alignment vs
  +/-2; cost stays bounded because the submap is re-voxeled at loop_voxel). ACCEPTED
  SCOPE (not a pending task): loop closure verifies only the single SC-best candidate
  (reloc already does multi-candidate). This is sufficient in distinctive scenes - the
  SC best is the correct match there (confirmed on the live map) - so multi-candidate
  loop verification is worth adding ONLY for repetitive/symmetric spaces (many near-
  identical rooms/corridors); add it then, not before.
- **Relocalization runs on its own timer (`reloc/rate`), not per scan**, in
  `relocTimerCb`: the synced callback only caches the latest scan. ACQUIRING runs
  global SC (`relocalizeGlobal`): it ICP-verifies the top `reloc/num_candidates`
  candidates (`queryCandidates`, both yaw signs) and accepts by **inlier ratio**
  (`inlierRatio >= reloc/min_inlier`, points within `reloc/inlier_dist`), NOT by
  fitness - fitness is a poor discriminator in multi-feature scenes. TRACKING runs
  `trackAgainstSubmap` against a cached map-frame submap (`snapshotKeyframesNear` +
  `assembleFromSnapshot`), rebuilt only when the predicted pose moves past
  `reloc/track_rebuild_dist`; same inlier gate. Because the map is frozen, the
  submap assembly + tracking ICP run OFF the mutex (only the cached scan,
  `map->odom`, and lock flags are touched under the lock), so the 20 Hz `map->odom`
  broadcast is never starved. `relocalize*` are pure; the node applies results via
  `setMapToOdom`. `/slam/relocalize` `use_guess=true` = guided ICP at the guess
  (`relocalizeLocal`); `use_guess=false` = global acquire; on success the node fills
  `Relocalize.locked_pose = map->odom * P_now`.
- **Robust tracking correction (do not drop).** TRACKING does NOT overwrite
  `map->odom` with the raw `trackAgainstSubmap` pose. Near a dominant planar
  surface (a table/monitor faced at close range) the live scan loses the
  far-field context and the point-to-point scan-to-map ICP is under-constrained:
  it slides along the surface and the raw pose teleports (validated on the
  grasp_can incident bag: `map->odom` jumped 1.5 m while the robot stood still and
  the front-end odom stayed smooth). `trackAgainstSubmap` returns the live-scan
  **point-to-plane info** (`RelocResult.obs_info`, mean-normalized, `[trans;rot]`),
  and `relocTimerCb` runs it through `computeTrackingCorrection` (`tracking_correction`):
  (1) project the correction off unobservable eigen-DOFs (`reloc/obs_eig_floor` /
  `obs_eig_ratio`), (2) clamp the per-tick step (`reloc/max_step_*`), (3) low-pass
  by `reloc/corr_gain`, (4) reject + coast on the smooth odom when the RAW jump
  exceeds `reloc/reject_*`. The front end is a tight iEKF LIO (smooth, locally
  accurate), so coasting on its odom between corrections is safe. A *rejected*
  correction still counts toward `reloc/lost_streak`, so a genuine loss
  (sustained low inlier OR sustained rejects) re-acquires globally. Layered on
  purpose: the clamp+gain bound every tick to ~`max_step_trans*corr_gain` even
  when the eigenvalue floor does not fire. Defaults are starting points - tune on a
  localization bag; floor/clamp/reject = 0 and `corr_gain` = 1.0 restore the legacy
  raw overwrite. Covered by `test/test_tracking_correction.cpp` (the pure rule) and
  `test/test_track_observability.cpp` (the Hessian reflects scene geometry).
- **Read-only localization + incremental append.** `load_map(read_only=true)` sets a
  node guard that hard-refuses `save_map` (the sole disk-write path), so localization
  NEVER modifies a map file. INCREMENTAL APPEND: a loaded keyframe's `odom_pose` is a
  PLACEHOLDER (the optimized map pose), so a naive "set MAPPING and append" makes the
  first new keyframe's bridge factor garbage. After a relocalization lock,
  `/slam/begin_incremental` (`Backend::beginIncremental`) re-bases each
  `odom_pose := map_to_odom^-1 * optimizedPose(id)` so the bridge factor is correct and
  `stepIntake`/`saveMap` stay consistent; it clears `read_only`. `/slam/reset`
  (`Backend::reset`) clears all state for a fresh mapping session. Core re-base +
  reset are covered by `test/test_incremental.cpp`; the node guard + `keyframe_poses`
  + the `begin_incremental` guard by the `test/backend_node_hooks.test` rostest.

## Build / test
```bash
# from catkin_ws/:
catkin build g1_msgs g1_slam_backend && source devel/setup.bash
catkin run_tests g1_slam_backend --no-deps && catkin_test_results build/g1_slam_backend
```
C++ gtests (ABI, pose_graph, scan_context, backend e2e) + a python launch/config
contract check. The full estimator is gated by a real G1 loopy bag. Node binary
g1_slam_backend_node; node name g1_slam_backend.

