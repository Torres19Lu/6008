# G1 Maps (g1_maps) 🗂️

**Data-only package**: the on-disk contract for named maps and the global cross-map
topology consumed by `g1_map_manager`. It defines the directory layout and the YAML
schemas; it has no runtime code and no nodes. The per-map runtime data under
`maps/<name>/` and the global `topology/` are not committed to git: a shared map's files
(geometry binaries + geometry YAML + overlays + the global `topology.yaml`) live on a
private Hugging Face dataset, fetched + sha256-verified at install; in live mapping the
YAML state is produced at runtime. This package is the single source of truth for the
map and topology format; `g1_map_manager` reads and writes this tree, and the SLAM
backend reads or writes one `geometry/` directory at a time.

## 🧱 What you get

- A directory contract for maps: a live `maps/<name>/geometry/` (backend `manifest.yaml`
  + manager `lineage.yaml` + clouds), a human `overlays/` layer (nav points + edit overlay
  + bounded `.history/`), and rare `snapshots/` restore points.
- `topology/topology.yaml`: the global cross-map graph (map names + captured gateways), the
  single source of truth for cross-map routing.
- An install-time asset fetch (`scripts/fetch_maps.py` + `config/maps_manifest.yaml`)
  that pulls the map files from HF and verifies each sha256.
- `CONTRACT.md`: the authoritative directory + YAML schema (manifest, lineage, nav points,
  edits, topology).

## 🗂️ On-disk contract

```
g1_maps/
  maps/<name>/
    geometry/        # the live map (atomic swap): manifest.yaml (backend) + lineage.yaml
                     # (manager) + pose_graph.g2o + scan_context.bin + keyframes/NNNNNN.pcd
    overlays/        # human layer: nav_points + {obstacle_mask,keyframe_trim,metadata}.yaml
                     # + .history/ (bounded, auto-pruned)
    snapshots/<YYYYMMDD-HHMMSS-slug>/   # rare full restore points (geometry/ + overlays/)
  topology/
    topology.yaml    # global graph (map names + gateways)
    .history/        # bounded, auto-pruned
```

A map is identified by NAME; `geometry/` holds the one live version. Goals and topology
edges reference the NAME and resolve to it. Saving (initial or increment) has the backend
write a fresh geometry which the manager commits by an atomic directory swap; the manager
auto-stamps a `pre-incremental` snapshot before each increment as the rollback point, and
the operator can stamp named milestones. `geometry/manifest.yaml` is backend-owned (the
manager never writes it); `geometry/lineage.yaml` is the manager's version sidecar. Edits
and nav points live under `overlays/`, applied over the live geometry, never baked into it.
See `CONTRACT.md` for the full schema.

## 🔒 Tracked vs fetched

Only the package scaffolding is tracked: docs, `CMakeLists.txt`/`package.xml`,
`config/maps_manifest.yaml`, and `scripts/fetch_maps.py`. Each runtime-data folder owns
the same comment-free local `.gitignore` (`*` + `!.gitignore`), and that tracked
`.gitignore` is what keeps the otherwise-empty folder in git: `maps/.gitignore` ignores
the whole `maps/<name>/` subtree, and `topology/.gitignore` ignores the `topology/`
subtree (the only two places map data lives), so no package-root `.gitignore` is needed. A shared
map is HF-hosted and fetched + sha256-verified at install (geometry binaries + geometry
YAML + overlays + `topology.yaml`, whatever `config/maps_manifest.yaml` pins); in live
mapping the YAML state is produced at runtime (with no shared map, the manager treats an
absent `topology.yaml` as an empty graph and writes it on the first gateway save). The
root repo `.gitignore` is never edited. To change the assets: push to HF, then bump `hf_revision` + the per-file `sha256`
in `config/maps_manifest.yaml`.

## 📦 First-time setup

Build the workspace (see `docs/INSTALL.md`), then fetch the map assets:

```bash
python3 scripts/fetch_maps.py   # from this package dir; reads config/maps_manifest.yaml
```

Idempotent: a file already present with the right sha256 is skipped. The private HF repo
needs a read token in the ambient `huggingface_hub` auth (`hf auth login`, or the
`HF_TOKEN` env var). The fetch is required only to run the map-management demos/tests
against the sample map; it is not needed to build.

## 🧪 Build

```bash
# from catkin_ws/:
catkin build g1_maps && source devel/setup.bash
```

A minimal catkin data package: it installs the `maps/` tree, the fetch script, and the
`topology/` folder when present; it generates no code and runs no node.
