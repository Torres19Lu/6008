# G1 Messages (g1_msgs) 📨

Shared interface definitions (messages and services) for the humis-g1 stack.
No runtime code: every other `g1_*` package depends on these types so the
contracts live in one place.

## 📨 Messages

| Message | Used by | Purpose |
|---------|---------|---------|
| `LocoStatus` | `g1_locomotion` | mode, battery, halted (soft-stop latch), error_code |
| `SemanticObject` / `SemanticObjectArray` | `g1_semantic_map` | persistent 3D object memory |
| `Detection3D` / `Detection3DArray` | `g1_perception` | per-frame 3D detections (`map`) |
| `NavPoint` / `NavPointArray` | `g1_semantic_map` / `g1_nav` | user-added navigation points |

## 🔧 Services

| Service | Used by | Purpose |
|---------|---------|---------|
| `SetMode` | `g1_locomotion` | set G1 locomotion mode: `damp` / `stand` / `start` |
| `SaveMap` / `LoadMap` / `Relocalize` | `g1_slam_backend` | map persistence + relocalization |
| `AddNavPoint` / `QueryNavPoint` | `g1_semantic_map` | add / query semantic nav points |

## 🎯 Actions

| Action | Used by | Purpose |
|--------|---------|---------|
| `NavigateTo.action` | `g1_nav` | navigate_to goal (Pose; reserved semantic id / nav-point); FSM feedback + outcome result |

## ✅ Test

```bash
# from catkin_ws/:
catkin build g1_msgs && source devel/setup.bash
catkin run_tests g1_msgs --no-deps && catkin_test_results build/g1_msgs
```
