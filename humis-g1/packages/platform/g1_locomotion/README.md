# G1 Locomotion (g1_locomotion) 🦿

Safety-critical bridge between ROS and the Unitree G1 onboard high-level
locomotion (`unitree_sdk2` `LocoClient`). We do NOT author a gait; the onboard
controller provides omnidirectional walking. We never switch the robot's FSM
automatically; it changes only via the user-invoked `/g1/set_mode` (or the
remote). The node streams velocity on demand and yields to the remote when idle.
This package maps velocity commands to the robot with safety limits, and
republishes the robot's state to ROS.

## 🔌 Interfaces

| Direction | Name | Type | Notes |
|-----------|------|------|-------|
| sub | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x`=vx, `linear.y`=vy, `angular.z`=vyaw |
| pub | `/joint_states` | `sensor_msgs/JointState` | 23 DOF incl. `waist_yaw_joint`; drives `robot_state_publisher` |
| pub | `/g1/body_imu` | `sensor_msgs/Imu` | frame `imu_in_pelvis` (configurable) |
| pub | `/g1/loco_status` | `g1_msgs/LocoStatus` | mode, `halted` (soft-stop latch), error_code; `battery_soc` best-effort (0.0) |
| srv | `/g1/halt` | `std_srvs/Trigger` | soft-stop + disarm: one zero velocity then silent (releases to the remote); does **NOT** damp |
| srv | `/g1/arm` | `std_srvs/Trigger` | arm ROS `/cmd_vel` streaming; moves only when also in a walk FSM with an active command |
| srv | `/g1/set_mode` | `g1_msgs/SetMode` | reserved manual FSM switch: `walk_motion` (FSM 500) / `lock_standing` / `damping` / `zero_torque` (last three: no balance -> robot falls) |

## 🛡️ Safety model

- **Latched-safe boot:** boots disarmed; issues no velocity until `/g1/arm`.
- **FSM is user-only (never auto-switched):** the FSM changes only via the
  user-invoked `/g1/set_mode` (manual) or the remote - no code path auto-switches
  it. The node streams `/cmd_vel` only while **armed** AND the robot is in an
  allowed walk FSM (`allowed_fsm_ids`); otherwise silent so the **remote keeps
  control** and the robot stands still when idle.
- **Clamp + deadzone + slew:** `/cmd_vel` is clamped to configured limits, a
  per-axis deadzone drops sub-threshold commands to zero, then it is slew-rate
  limited by configured acceleration.
- **Send-on-demand (TTL + watchdog backstops):** `SetVelocity` is issued only while
  driving or decelerating; idle -> silent. On stale/zero `/cmd_vel` the node
  decelerates to zero then goes silent. Every `SetVelocity` carries a short
  `command_ttl` so the robot self-stops if this node dies.
- **Soft-stop (`/g1/halt`):** one `SetVelocity(0)` (prompt stop) then silent +
  disarm (releases to the remote). Re-enable with `/g1/arm`. It does **NOT** damp.
- **WARNING - no-balance modes:** `set_mode` `zero_torque`/`damping`/`lock_standing`
  (FSM 0/1/4) have **no balance control - the robot FALLS**; use only when
  physically supported, never as a stop. The hardware e-stop button is the true
  power-kill.

All tunables live in `config/locomotion.yaml`; the network interface comes from
`$G1_NETWORK_INTERFACE` via the launch file.

## 🚀 Build / test / run

```bash
# from catkin_ws/:
catkin build g1_msgs g1_locomotion && source devel/setup.bash
catkin run_tests g1_locomotion --no-deps && catkin_test_results build/g1_locomotion
roslaunch g1_locomotion locomotion.launch
```

## 🤖 Live gate (needs the robot, open space, e-stop in hand)

Operator readies the robot to walk via the remote (FSM 500), then: `/g1/arm` ->
`/cmd_vel` translates (vx fwd, vy left, yaw CCW) -> releasing leaves it standing
still (no march) -> the remote still works while armed + idle -> `/g1/halt`
soft-stops. `/g1/set_mode` is the reserved manual FSM switch. Also read-only
state + RViz live `waist_yaw_joint`. Full procedure in CLAUDE.md invariants
and the live-gate section below.
