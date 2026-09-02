# g1_maps on-disk contract

Data-only package: this document is the authoritative directory layout and YAML
schema that g1_map_manager reads and writes. No runtime code lives here. None of
the map state is committed to git: `maps/.gitignore` ignores the whole
`maps/<name>/` subtree (keeping only its own tracked `.gitignore`) and `topology/.gitignore`
ignores the `topology/` subtree (keeping only its `.gitignore`). A shared map's
files are fetched from a private Hugging Face dataset and verified by sha256 at
install time (see `scripts/fetch_maps.py` + `config/maps_manifest.yaml`): the
geometry binaries (`*.pcd`, `*.g2o`, `*.bin`), the geometry YAML, the overlays, and
the global `topology.yaml` -- whatever the manifest pins, each restored to its
package-relative path. In live mapping that YAML state is produced/updated at
runtime (with no shared map, an absent `topology.yaml` reads as an empty graph).
Only the package scaffolding
(docs, build files, the manifest, the fetch script, and the two local `.gitignore`
files under `maps/` and `topology/`) is tracked.

## Directory layout

A map has exactly three subdirectories, one per lifecycle: machine-produced
`geometry/` (replaced atomically), human-authored `overlays/` (mutable, persists
across geometry swaps), and rare `snapshots/` (full restore points).

```
g1_maps/
  maps/
    <map_name>/
      geometry/                  # the live map; written by the backend, swapped atomically
        manifest.yaml            # BACKEND-owned SLAM facts (the manager never writes it)
        lineage.yaml             # MANAGER-owned version metadata (sidecar)
        pose_graph.g2o           # optimized keyframe poses + edges (SE3:QUAT text)
        scan_context.bin         # Scan Context descriptors (f64; ring keys recomputed on load)
        keyframes/NNNNNN.pcd      # per-keyframe clouds, base_link frame
      overlays/                  # human-authored, mutable, geometry-independent
        nav_points.yaml          # map-scoped user nav points (keyframe-anchored)
        obstacle_mask.yaml       # non-destructive edit overlay (map-frame coords)
        keyframe_trim.yaml
        metadata.yaml
        .history/                # bounded, auto-pruned: <base>.<ts>.yaml (last N per file)
      snapshots/
        <YYYYMMDD-HHMMSS-slug>/  # full frozen restore point; same shape as <map_name>/
          geometry/{manifest.yaml, lineage.yaml, pose_graph.g2o, scan_context.bin, keyframes/}
          overlays/{nav_points.yaml, obstacle_mask.yaml, keyframe_trim.yaml, metadata.yaml}
  topology/
    topology.yaml                # global cross-map graph (map names + gateways)
    .history/                    # bounded, auto-pruned: topology.<ts>.yaml (last N)
```

A map is identified by NAME. `geometry/` holds the one live version; goals and
topology edges reference the NAME and resolve to it (no version pinning). Saving
(initial mapping or an increment) has the backend write a fresh `geometry.tmp/`
which the manager commits by an atomic directory swap, after adding its
`lineage.yaml`. Snapshots are rare: the manager auto-stamps one
`<ts>-pre-incremental` snapshot before each incremental run (the rollback point),
and the operator can stamp named milestones; `snapshots/<id>/` is structurally
identical to `<map_name>/` minus `snapshots/`, so restore is a copy-back.

## geometry/manifest.yaml (backend-owned, never written by the manager)

The SLAM backend (`g1_slam_backend`) writes this in `save_map` and reads + validates
it in `load_map` with its own flat key:value parser. The manager treats it as
read-only (it may parse `num_keyframes` for display). Schema (verbatim):

```
format_version: <int>
num_keyframes: <int>
keyframe_voxel: <float>
sc_num_rings: <int>
sc_num_sectors: <int>
sc_max_range: <float>
sc_min_range: <float>
sc_height_offset: <float>
```

## geometry/lineage.yaml (manager-owned sidecar)

The version/workflow metadata the backend has no concept of. Sits next to
`manifest.yaml`, so it travels into a snapshot via the same copy. Written into
`geometry.tmp/` just before the atomic commit.

```yaml
name: <map_name>
created_at: <ISO-8601 timestamp>
label: <short human label, e.g. "extend west wing">
reason: initial | incremental | edit | imported
parent: <snapshot id this geometry derived from, or "">
```

`parent` walked across `snapshots/*/geometry/lineage.yaml` gives a readable lineage
(`map_cli history <name>`), replacing the old opaque `v0001/v0002` listing.

## overlays/nav_points.yaml (mutable, map-scoped, keyframe-anchored)

A per-map store of user-authored navigation points (base, grasp point, delivery
point, ...). Under `overlays/` (not baked into geometry) so it can be authored live
during mapping and edited offline.

```yaml
map_name: <map_name>
points:
  - name: base                 # unique within the map
    anchor_keyframe_id: <uint>
    anchor_relative_pose:      # T_keyframe_navpoint
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    source: user
```

The world pose is resolved as `keyframe_world_pose(anchor) * anchor_relative_pose`
from the live keyframe poses (live: `/slam/keyframe_poses`; saved:
`geometry/pose_graph.g2o` `VERTEX_SE3:QUAT`). A point authored early in mapping
stays correct after loop closure re-optimizes the graph. Writable in
mapping/incremental/editing, refused in localization (query still allowed). Every
write rolls the prior file into `overlays/.history/` (bounded to the last N).

## overlays/ edit overlay (mutable, map-level, applied over geometry)

Non-destructive: `geometry/` files are never modified. The overlay is applied at
load (by the manager-side keyframe relay) over the live geometry. All coordinates
are in the map frame.

```yaml
# obstacle_mask.yaml
deleted_voxels:                # remove points within delete_radius of each
  - [x, y, z]
```

```yaml
# keyframe_trim.yaml
suppressed_keyframes: [<uint>, ...]   # drop whole keyframes by id
crops:                                # keep-inside polygons; a point outside ALL
  - - [x, y]                          # crop polygons is dropped (applied only when
    - [x, y]                          # at least one crop exists)
    - [x, y]
```

```yaml
# metadata.yaml -- name/description/extent overrides for the registry view
name: <display name>
description: <string>
extent: [xmin, ymin, xmax, ymax]
```

## overlays/.history/ and topology/.history/ (bounded local undo)

Every save of a mutable human-authored file (overlays + topology) first copies the
current file to `<base>.<ts>.yaml` in the sibling `.history/`, then prunes that base
to the newest N (`overlay_history_keep` / `topology_history_keep`, default 5). This
is the cross-session fine-grained undo; geometry rollback is via `snapshots/`.

## topology/topology.yaml (global)

Nodes are map names; edges are captured gateways. Single source of truth for
cross-map routing and consistency checks. Lives in its own `topology/` folder
(mirroring how `maps/` groups per-map data).

```yaml
version: 1
maps: [<name>, ...]
gateways:
  - from_map: <name>
    to_map: <name>
    pose_in_from: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}
    pose_in_to:   {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}
    label: <string>
    source: captured
```

Edges are bidirectional; the manager picks the departure pose (in the map being
left) and the arrival seed (in the map being entered) by traversal direction. Both
poses are captured from a successful relocalization in each map (operator-driven
begin/load/commit via `/map_manager/gateway_capture`), not hand-typed. A map's
neighbor edges are DERIVED from this file.
