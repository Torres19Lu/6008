# g1_locomotion - agent guide

Safety-critical bridge to the G1 onboard high-level locomotion (`unitree_sdk2`
`LocoClient`). Three layers: a pure SDK-free/ROS-free core (`loco_safety`,
`joint_map`, unit-tested), an SDK wrapper (`loco_bridge`, thread-safe), and a ROS
node (`locomotion_node`). Downstream: `g1_local_planner`/`g1_nav` stream
`/cmd_vel`; `robot_state_publisher` consumes `/joint_states`.

## Invariants (do not break)
- **Latched-safe + silent-when-idle:** the bridge boots `halted_=true` (disarmed)
  and issues NO `SetVelocity` until `/g1/arm`, a non-zero `/cmd_vel` arrives, AND
  the robot is in a walk FSM. While idle/stopped (or disarmed, or off a walk FSM)
  it stays SILENT so the Unitree handheld remote keeps control of the shared
  `"sport"` service. `/cmd_vel` before arming is stored, not sent.
- **FSM is user-only; never auto-switched.** The onboard FSM changes ONLY via the
  user-invoked `/g1/set_mode` (manual `SetFsmId`): `walk_motion`=`Start()`(500),
  `lock_standing`(4), `damping`(1), `zero_torque`(0). `LocoBridge::setMode` is the
  ONLY `SetFsmId` caller; NO startup/timer/watchdog/shutdown/`controlStep`/`halt`/
  `arm` path ever switches the FSM. Do NOT add an autonomous FSM caller, and do
  NOT call `SwitchToUserCtrl`/`SetBalanceMode`/`ContinuousGait`/`Squat`/`Sit`.
- **`/g1/set_mode` does NOT arm; `/g1/arm` + `/g1/halt` own streaming.** Switch and
  arm are decoupled so forcing a walk FSM never re-couples to translation: `/g1/arm`
  clears the halt latch (enable streaming, no FSM change); `/g1/halt` soft-stops
  (one `SetVelocity(0)`, then latch + silent; no FSM change, never `Damp()`).
  `set_mode walk_motion` only switches FSM (call `/g1/arm` to drive); the three
  no-balance modes also disarm (the robot is dropping).
- **Translation needs an operator-prepared walk FSM.** Live finding: a programmatic
  `Start()`=`SetFsmId(500)` re-enters a static balance stand that does NOT
  translate; the gamepad-prepared FSM 500 translates on `SetVelocity` and stands
  still when idle. `ContinuousGait`(`SetBalanceMode(1)`) translates but marches in
  place (rejected). So we never force gait; the operator readies a walk FSM (remote,
  or `set_mode walk_motion`) and `controlStep` streams on demand.
- **Walk-FSM whitelist:** `controlStep` streams `SetVelocity` ONLY when the cached
  FSM (from `pollStatus`, refreshed immediately by `setMode`) is in config
  `allowed_fsm_ids` (e.g. `{500, 801}`); otherwise silent (yields to the remote).
- **No-balance modes -> the robot FALLS.** `zero_torque`(0)/`damping`(1)/
  `lock_standing`(4) have NO balance control; deliberate, operator-supported
  actions, never a stop. The hardware e-stop button is the true power-kill.
- **Send-on-demand; TTL + watchdog are backstops:** `controlStep` issues
  `SetVelocity(..., duration=command_ttl)` ONLY while armed AND actively driving
  (or still decelerating to a stop) AND in a walk FSM; once disarmed, idle and
  stopped, or off a walk FSM it issues nothing (silent). On a stale or zero
  `/cmd_vel` it slews to zero (controlled decel) then goes silent (node watchdog);
  `command_ttl` is the process-crash backstop (SDK self-stop). Do NOT restore
  unconditional per-tick `SetVelocity` - it stomps the remote.
- **SetVelocity, not Move:** `Move` is a thin SDK wrapper that only allows
  duration `1.0s` or `864000s` (864000 == continuous == NO SDK self-stop). We
  call `SetVelocity` directly with the short, configurable `command_ttl` (same
  underlying API `ROBOT_API_ID_LOCO_SET_VELOCITY`) so the self-stop window stays
  tight and tunable. Do not switch to `Move()`/`SwitchMoveMode` (hidden state,
  wrong TTL).
- **Lock order:** the only nested locks are `client_mutex_` then `cmd_mutex_`
  (in `setMode`). `controlStep`/`halt`/`pollStatus` take `cmd_mutex_` and release
  it before (or after, sequentially, never nested with) `client_mutex_`. Do not
  introduce a `cmd_mutex_`-held-while-waiting-`client_mutex_` path (deadlock).
- **Joint map is data, not code:** the 23-DOF motor-index -> URDF-name table is
  in `config/locomotion.yaml`. It was derived from `unitree::robot::g1::JointIndex`
  (SDK `dds_wrapper/robots/g1/defines.h`) + `g1_description/.../g1_23dof_mode_10.urdf`.
  23-DOF uses SDK indices 0-12, 15-19, 22-26. Edit the YAML, never hardcode.
- **Deadzone is data:** the per-axis `/cmd_vel` deadzone (`deadzone_vx/vy/vyaw`)
  lives in `config/locomotion.yaml`, set at/near the robot's minimum effective
  velocity (hard cutoff, pass-through above; `applyDeadzone` in `loco_safety`).
  Below it a command reads as zero -> idle -> silent. Edit the YAML, never hardcode.
- **Blocking FSM polls at low rate:** `LocoClient.GetFsmId` blocks (DDS req/resp);
  poll only at `status_rate` (~2 Hz) under the `AsyncSpinner`. Never per-frame.
- **Short RPC timeout, modest control rate:** every `LocoClient` call is a blocking
  request/response RPC sharing one `client_mutex_`. Keep `client_timeout` small
  (0.2 s) and `< command_ttl`: a long timeout (the old 10.0) turns one WiFi-dropped
  round-trip into a multi-second freeze of the control + status loop (SDK err 3104
  = `UT_ROBOT_ERR_CLIENT_API_TIMEOUT`; SDK default is 1 s). Keep `control_rate`
  modest (~10 Hz) - `SetVelocity` self-stops after `command_ttl`, so flooding RPCs
  over WiFi only raises loss. See the plan's live-gate results (2026-06-15).
- **`/g1/arm` arms; `LocoStatus.halted` reports the latch.** Arming clears the
  latch (no SDK call, no FSM change); streaming is still gated by the walk-FSM
  whitelist + an active `/cmd_vel`, so arming off a walk FSM moves nothing.
- **IMU quaternion order is `[w,x,y,z]`** from the SDK; mapped explicitly to
  `sensor_msgs/Imu` in the node. `imu_frame` (default `imu_in_pelvis`) and this
  order are verified at the live gate.
- **`battery_soc`** has no source in hg `LowState_`; published as 0.0 (best-effort).
- **Design origin:** the control model (silent-when-idle, on-demand streaming,
  walk-FSM whitelist, deadzone, no automatic FSM switching) derives from the
  SDK headers, our URDF, and the live-gate findings; no code was
  copied from any other package.

## Build / test / run
```bash
# from catkin_ws/:
catkin build g1_msgs g1_locomotion && source devel/setup.bash
catkin run_tests g1_locomotion --no-deps && catkin_test_results build/g1_locomotion
roslaunch g1_locomotion locomotion.launch [interface:=eth0]
```

## Live gate (needs the robot)
Operator-driven, open space, e-stop in hand. Operator readies the robot to walk
via the remote (FSM 500), then: `/g1/arm` -> `/cmd_vel` translates (vx fwd, vy
left, yaw CCW) -> releasing leaves it standing still (no march) -> the remote
still works while armed+idle -> `/g1/halt` soft-stops. `/g1/set_mode` is the
reserved manual FSM switch. Safety limits and allowed FSM IDs are in
`config/locomotion.yaml`; the control model is described in the Invariants
section above.
