# g1_maps - agent guide

Data-only package: the on-disk contract for named maps + the global cross-map
topology. No runtime code, no nodes, no message generation. g1_map_manager
reads/writes this tree; the SLAM backend reads/writes a single geometry/ directory
at a time.

## Boundary (do not break)
- DATA ONLY. No C++/Python nodes here: schema, sample/test assets, and the fetch
  script. Logic lives in g1_map_manager.
- Three subdirs per map, one lifecycle each: geometry/ (machine-produced, swapped
  atomically), overlays/ (human layer, mutable, persists across geometry swaps),
  snapshots/ (rare full restore points). Saving (initial or increment) has the
  backend write geometry.tmp/ which the manager commits by an atomic dir swap;
  before each increment the manager auto-stamps a <ts>-pre-incremental snapshot.
- TWO metadata files in geometry/, strict ownership: manifest.yaml is BACKEND-owned
  (flat key:value, hand-parsed by g1_slam_backend save_map/load_map); the manager
  NEVER writes it (it may read num_keyframes). lineage.yaml is the MANAGER-owned
  sidecar (name/created_at/label/reason/parent). Do NOT make the manager serialize
  manifest.yaml -- it would break the backend loader.
- NONE of the map state is in git. maps/.gitignore (`* !.gitignore`)
  ignores the whole maps/<name>/ subtree; topology/.gitignore (`* !.gitignore`)
  ignores the topology/ subtree. Each tracked local .gitignore is also what keeps its
  otherwise-empty folder in git (no .gitkeep needed). Only scaffolding is tracked: docs,
  CMakeLists/package.xml, config/maps_manifest.yaml, scripts/fetch_maps.py, the two
  local .gitignore files. A shared map is HF-hosted (fetched + sha256-verified at
  install) per config/maps_manifest.yaml: geometry binaries (*.pcd, *.g2o, *.bin) +
  geometry YAML (manifest/lineage) + overlays + the global topology.yaml. In live
  mapping that YAML is produced/updated at runtime. Local .gitignores are
  comment-free; there is no package-root
  g1_maps/.gitignore (maps/ + topology/ cover all map data) and do NOT edit the root
  repo .gitignore.
- topology/topology.yaml is the single source of truth for cross-map routing;
  per-map neighbors are derived from it. Not committed -- fetched from HF for a
  shared map (it is in the manifest), else the manager reads an absent file as an
  empty graph and writes it on the first gateway save. CMake installs the topology/
  dir only when present.

## Layout
- maps/<name>/geometry/{manifest.yaml(backend), lineage.yaml(manager), pose_graph.g2o, scan_context.bin, keyframes/NNNNNN.pcd}
- maps/<name>/overlays/{nav_points,obstacle_mask,keyframe_trim,metadata}.yaml + .history/
- maps/<name>/snapshots/<YYYYMMDD-HHMMSS-slug>/{geometry/,overlays/}
- topology/topology.yaml + topology/.history/   # global graph + bounded history
- CONTRACT.md                        # full directory + YAML schema (authoritative)
- config/maps_manifest.yaml          # HF repo + revision + per-file sha256
- scripts/fetch_maps.py              # HF fetch + verify

## Build / fetch
```bash
# from catkin_ws/:
catkin build g1_maps && source devel/setup.bash
python3 src/g1_maps/scripts/fetch_maps.py   # pull + verify map files from HF
```
Installs the maps/ tree + fetch script (+ topology/ when present); no codegen.
