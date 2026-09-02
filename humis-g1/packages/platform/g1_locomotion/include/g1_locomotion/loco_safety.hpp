#ifndef G1_LOCOMOTION_LOCO_SAFETY_HPP
#define G1_LOCOMOTION_LOCO_SAFETY_HPP

#include <cstdint>
#include <vector>

#include "g1_locomotion/loco_types.hpp"

namespace g1_locomotion {

// Clamp each axis of cmd to its limit (magnitudes; a negative limit -> 0). vx is
// asymmetric: forward to vx_max, backward to vx_back_max (0 -> symmetric = vx_max);
// vy/vyaw are symmetric +/- their max.
Velocity clampVelocity(const Velocity& cmd, const VelocityLimits& lim);

// Per-axis hard cutoff deadzone: if |axis| < the matching deadzone the axis
// becomes 0; otherwise it passes through unchanged (no rescaling, so a commanded
// velocity keeps its magnitude). A negative/zero deadzone disables that axis; a
// non-finite input axis becomes 0.
Velocity applyDeadzone(const Velocity& cmd, const VelocityDeadzone& dz);

// Step `current` toward `target` by at most accel*dt per axis (slew limit).
// dt <= 0 returns current unchanged; never overshoots target.
Velocity slewVelocity(const Velocity& current, const Velocity& target,
                      const AccelLimits& acc, double dt);

// True when the newest command is older than `timeout` (a stale /cmd_vel).
// A negative age (clock skew) is treated as fresh -> false.
bool isCommandStale(double age_seconds, double timeout);

// True if fsm_id is in the walk whitelist (the FSMs where the robot accepts
// velocity and stays balanced, e.g. 500/801). Empty list or unknown id -> false.
bool fsmAllowed(int fsm_id, const std::vector<int>& allowed);

// Map a G1 onboard FSM id to a g1_msgs/LocoStatus MODE_* value:
//   0 (ZeroTorque)->MODE_ZERO_TORQUE, 1 (Damp)->MODE_DAMP,
//   4 (StandUp)->MODE_STAND, 500 (Start)->MODE_WALK, else MODE_UNKNOWN.
std::uint8_t fsmIdToLocoMode(int fsm_id);

}  // namespace g1_locomotion
#endif  // G1_LOCOMOTION_LOCO_SAFETY_HPP
