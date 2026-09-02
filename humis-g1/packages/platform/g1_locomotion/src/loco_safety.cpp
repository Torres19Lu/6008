#include "g1_locomotion/loco_safety.hpp"

#include <algorithm>
#include <cmath>

#include <g1_msgs/LocoStatus.h>

namespace g1_locomotion {
namespace {

double clampAxis(double v, double limit) {
  if (!std::isfinite(v)) return 0.0;  // reject NaN/inf from a misbehaving cmd
  const double m = std::max(0.0, limit);
  return std::max(-m, std::min(m, v));
}

double clampAxisAsym(double v, double fwd, double back) {
  if (!std::isfinite(v)) return 0.0;  // reject NaN/inf from a misbehaving cmd
  const double hi = std::max(0.0, fwd);
  const double lo = -((back > 0.0) ? back : hi);  // 0 -> symmetric (= fwd)
  return std::max(lo, std::min(hi, v));
}

double deadzoneAxis(double v, double dz) {
  if (!std::isfinite(v)) return 0.0;  // reject NaN/inf from a misbehaving cmd
  const double d = std::max(0.0, dz);
  return (std::abs(v) < d) ? 0.0 : v;  // pass through at/above the threshold
}

double slewAxis(double cur, double tgt, double accel, double dt) {
  const double max_step = std::max(0.0, accel) * dt;
  const double delta = tgt - cur;
  if (std::abs(delta) <= max_step) return tgt;
  return cur + std::copysign(max_step, delta);
}

}  // namespace

Velocity clampVelocity(const Velocity& cmd, const VelocityLimits& lim) {
  Velocity out;
  out.vx = clampAxisAsym(cmd.vx, lim.vx_max, lim.vx_back_max);  // asym fwd/back
  out.vy = clampAxis(cmd.vy, lim.vy_max);
  out.vyaw = clampAxis(cmd.vyaw, lim.vyaw_max);
  return out;
}

Velocity applyDeadzone(const Velocity& cmd, const VelocityDeadzone& dz) {
  Velocity out;
  out.vx = deadzoneAxis(cmd.vx, dz.vx);
  out.vy = deadzoneAxis(cmd.vy, dz.vy);
  out.vyaw = deadzoneAxis(cmd.vyaw, dz.vyaw);
  return out;
}

Velocity slewVelocity(const Velocity& current, const Velocity& target,
                      const AccelLimits& acc, double dt) {
  if (dt <= 0.0) return current;
  Velocity out;
  out.vx = slewAxis(current.vx, target.vx, acc.ax_max, dt);
  out.vy = slewAxis(current.vy, target.vy, acc.ay_max, dt);
  out.vyaw = slewAxis(current.vyaw, target.vyaw, acc.ayaw_max, dt);
  return out;
}

bool isCommandStale(double age_seconds, double timeout) {
  if (age_seconds < 0.0) return false;
  return age_seconds > timeout;
}

bool fsmAllowed(int fsm_id, const std::vector<int>& allowed) {
  for (int a : allowed) {
    if (fsm_id == a) return true;
  }
  return false;
}

std::uint8_t fsmIdToLocoMode(int fsm_id) {
  switch (fsm_id) {
    case 0:
      return g1_msgs::LocoStatus::MODE_ZERO_TORQUE;
    case 1:
      return g1_msgs::LocoStatus::MODE_DAMP;
    case 4:
      return g1_msgs::LocoStatus::MODE_STAND;
    case 500:
      return g1_msgs::LocoStatus::MODE_WALK;
    default:
      return g1_msgs::LocoStatus::MODE_UNKNOWN;
  }
}

}  // namespace g1_locomotion
