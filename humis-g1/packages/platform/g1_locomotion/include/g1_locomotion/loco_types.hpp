#ifndef G1_LOCOMOTION_LOCO_TYPES_HPP
#define G1_LOCOMOTION_LOCO_TYPES_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace g1_locomotion {

// Motor slots in the unitree_hg LowState_ message (motor_state array length).
constexpr std::size_t kLowStateMotorCount = 35;

// Planar base velocity (G1 omnidirectional): vx, vy [m/s]; vyaw [rad/s].
struct Velocity {
  double vx = 0.0;
  double vy = 0.0;
  double vyaw = 0.0;
};

// Per-axis velocity magnitude limits (>= 0). vx may be asymmetric: vx_max caps
// forward, vx_back_max caps backward magnitude (backward is blind, so usually
// slower). vx_back_max == 0 means "symmetric" (backward also capped at vx_max).
struct VelocityLimits {
  double vx_max = 0.0;
  double vy_max = 0.0;
  double vyaw_max = 0.0;
  double vx_back_max = 0.0;  // backward speed cap [m/s]; 0 -> symmetric (= vx_max)
};

// Per-axis acceleration (slew-rate) limits in units/s (>= 0).
struct AccelLimits {
  double ax_max = 0.0;
  double ay_max = 0.0;
  double ayaw_max = 0.0;
};

// Per-axis deadzone magnitude (>= 0): a /cmd_vel axis whose magnitude is below
// this is treated as zero (no command). Set at/near the robot's minimum
// effective velocity so sub-threshold commands neither stomp the manual remote
// nor twitch the gait. 0 disables the deadzone for that axis.
struct VelocityDeadzone {
  double vx = 0.0;
  double vy = 0.0;
  double vyaw = 0.0;
};

// One row of the motor-index -> URDF-joint map (SDK
// unitree::robot::g1::JointIndex + URDF joint order). sdk_index addresses LowState_.motor_state().
struct JointMapEntry {
  std::string name;    // URDF joint name, e.g. "left_hip_pitch_joint"
  int sdk_index = -1;  // index into LowState_.motor_state() (0..34)
};

}  // namespace g1_locomotion
#endif  // G1_LOCOMOTION_LOCO_TYPES_HPP
