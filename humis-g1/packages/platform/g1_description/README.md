# G1 Description (g1_description) 🤖

URDF, TF frames, and `robot_state_publisher` bring-up for the Unitree G1 (23-DOF) - the description layer of the humis-g1 stack.

## 📦 First-time setup
Meshes are not in git; fetch them from Hugging Face (one read-token login, then pull):
```bash
~/miniconda3/envs/humis_g1_env_py310/bin/hf auth login
~/miniconda3/envs/humis_g1_env_py310/bin/python scripts/fetch_meshes.py
```

## 🚀 Run
```bash
roslaunch g1_description display.launch animate:=true   # RViz; torso sweeps via waist_yaw
roslaunch g1_description description.launch              # robot_description + RSP + base_footprint
```

## 🌳 Frames
`base_link` (identity to `pelvis`, the root) -> URDF chain via the dynamic `waist_yaw_joint` -> `torso_link` -> `{mid360_link, d435_link -> camera_link, head, arms}`. `base_footprint` is the live ground projection; everything else is published by `robot_state_publisher`.

## ✅ Test
```bash
catkin build g1_description --catkin-make-args run_tests && catkin_test_results build/g1_description
```

More: `../docs/INSTALL.md` (setup).
