# g1_description - notes for Claude

ROS Noetic **description layer**: the G1 URDF, TF frames, and `robot_state_publisher` bring-up. No algorithms live here.

## ⚠️ Meshes are NOT in git
The 26 `*.STL` live on the private HF dataset `humis-g1/g1-description-meshes`, pinned in `config/meshes_manifest.yaml`, pulled by `scripts/fetch_meshes.py` into `meshes/` (which `meshes/.gitignore` ignores). Never `git add` a mesh. To change them: push to HF, then bump `hf_revision` + per-file `sha256` in the manifest. See `../docs/INSTALL.md`.

## Frame design (do not break)
- `base_link` = `pelvis`: fixed identity, the REP-105 root. The URDF `world`/floating joint stays commented out.
- `base_footprint`: published at runtime by `base_footprint_publisher` (ground projection, z=0, yaw-only). NOT a URDF joint.
- `waist_yaw_joint` (pelvis -> torso_link) is dynamic (+/-2.618 rad); `mid360_link`/`d435_link` are fixed on `torso_link`. `d435_link -> camera_link` defaults to identity (overridden once the camera extrinsic is measured).
- `g1.urdf.xacro` `xacro:include`s the vendored `g1_23dof_mode_10.urdf` and adds `base_link` + the camera bridge.

## Conventions
- C++ only; no hardcoded frames/rates - read from `config/g1_description.yaml` (private params).
- Tests: each test file owns its `main()` (do not link `gtest_main`). Unit via `catkin_add_gtest`, integration via `add_rostest_gtest`.
- Per-dir local `.gitignore`; never edit the root `.gitignore`.
- `package.xml` authorship matches `g1_msgs`.

## Build & test
```bash
catkin build g1_description
catkin build g1_description --catkin-make-args run_tests && catkin_test_results build/g1_description   # URDF-only; passes without meshes
```
