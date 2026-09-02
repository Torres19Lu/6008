// footprint_safety.cpp -- footprint collision check + clear-distance implementation.
#include "g1_local_planner/core/footprint_safety.h"

#include <cmath>

namespace g1_local_planner {

// Tests the full footprint DISKS (each circle's area, optionally dilated by
// footprint_collision_margin) against true-lethal cells (lethal_value, default 100).
// Since the published costmap now distinguishes inscribed(99) from lethal(100), the
// filter checks the actual body geometry vs real obstacles only, so the robot may
// approach to ~circle_radius (honest zero margin). The resolution floor is ~half a
// cell. The MPC soft obstacle term still samples circle centres (a soft gradient
// nudge); the HARD guarantee is this area check.
bool FootprintSafety::worldPointLethal(double wx, double wy,
                                       const CostmapView& cm) const {
  int mx = 0, my = 0;
  if (!cm.worldToMap(wx, wy, mx, my)) return true;  // off-grid = unsafe
  const int v = static_cast<int>(cm.at(mx, my));
  if (v == -1) return cfg_.treat_unknown_as_obstacle;  // unknown cell
  return v >= cfg_.lethal_value;
}

bool FootprintSafety::worldDiskLethal(double cx, double cy, double r,
                                      const CostmapView& cm) const {
  int mcx = 0, mcy = 0;
  if (!cm.worldToMap(cx, cy, mcx, mcy)) return true;  // disk centre off-grid = unsafe
  const double res = cm.resolution();
  if (res <= 0.0) return true;
  const double r2 = r * r;
  const int span = static_cast<int>(std::ceil(r / res)) + 1;
  for (int dy = -span; dy <= span; ++dy) {
    for (int dx = -span; dx <= span; ++dx) {
      const int mx = mcx + dx;
      const int my = mcy + dy;
      // Cell-centre world coords (matches CostmapView's convention).
      const double wx = cm.originX() + (static_cast<double>(mx) + 0.5) * res;
      const double wy = cm.originY() + (static_cast<double>(my) + 0.5) * res;
      if ((wx - cx) * (wx - cx) + (wy - cy) * (wy - cy) > r2) continue;
      if (worldPointLethal(wx, wy, cm)) return true;  // off-grid/unknown/>= lethal_value
    }
  }
  return false;
}

bool FootprintSafety::poseInCollision(const Pose2D& p, const CostmapView& cm) const {
  if (!cm.valid()) return true;
  const double c = std::cos(p.yaw);
  const double s = std::sin(p.yaw);
  for (const auto& circle : fp_.circles()) {
    const double cx = p.x + c * circle.x_body - s * circle.y_body;
    const double cy = p.y + s * circle.x_body + c * circle.y_body;
    const double r = circle.radius + cfg_.footprint_collision_margin;
    if (worldDiskLethal(cx, cy, r, cm)) return true;
  }
  return false;
}

double FootprintSafety::clearDistance(const Pose2D& current,
                                      const std::vector<Pose2D>& predicted,
                                      const CostmapView& cm) const {
  if (!cm.valid()) return 0.0;
  double arc = 0.0;
  Pose2D prev = current;
  // The swept check tests footprint circle centers at discrete poses along each
  // predicted segment. To never skip a thin (one-cell) obstacle, the spacing between
  // consecutive checked poses must stay <= one cell, so the per-segment subsample
  // count is the LARGER of the configured floor and ceil(seg / resolution). The
  // swept_subsamples knob can only raise the density, never drop the stride above a
  // cell -- so the "thin obstacle never skipped" guarantee holds for any config /
  // MPC step size, not just the small default per-cycle motion.
  const double res = cm.resolution();
  const int sub_floor = cfg_.swept_subsamples < 1 ? 1 : cfg_.swept_subsamples;
  for (const Pose2D& next : predicted) {
    const double seg = std::hypot(next.x - prev.x, next.y - prev.y);
    int sub = sub_floor;
    if (res > 0.0 && seg > 0.0) {
      const int by_res = static_cast<int>(std::ceil(seg / res));
      if (by_res > sub) sub = by_res;
    }
    for (int i = 1; i <= sub; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(sub);
      Pose2D mid;
      mid.x = prev.x + t * (next.x - prev.x);
      mid.y = prev.y + t * (next.y - prev.y);
      // Interpolate yaw on the shortest arc.
      mid.yaw = prev.yaw + t * wrapToPi(next.yaw - prev.yaw);
      if (poseInCollision(mid, cm)) {
        return arc + t * seg;
      }
    }
    arc += seg;
    prev = next;
  }
  return kClearInf;
}

SafetyResult FootprintSafety::filter(const Pose2D& current, const MpcResult& mpc,
                                     const CostmapView& cm, double acc_lim_x,
                                     double control_dt) const {
  SafetyResult r;
  r.command = mpc.applied;
  if (!cfg_.enabled) return r;

  if (!cm.valid() || poseInCollision(current, cm)) {
    r.command = Twist2D{};       // hold
    r.intervened = true;
    r.state = ControllerState::STUCK;
    return r;
  }

  const double d = clearDistance(current, mpc.predicted, cm);
  if (d >= kClearInf) return r;  // fully clear -> pass unchanged

  const double d_usable = std::max(0.0, d - cfg_.brake_margin);
  const double a = acc_lim_x > 0.0 ? acc_lim_x : 1.0;
  const double v_safe = std::sqrt(2.0 * a * d_usable);

  const double vx = mpc.applied.vx;
  if (std::fabs(vx) <= v_safe + 1e-9) {
    return r;                    // command already brake-safe
  }

  r.intervened = true;
  if (v_safe <= 1e-6) {
    // Brake toward zero, slew-limited (decelerate, do not jump).
    const double step = a * control_dt;
    double nv = vx;
    if (nv > 0.0) nv = std::max(0.0, nv - step);
    else if (nv < 0.0) nv = std::min(0.0, nv + step);
    r.command.vx = nv;
    // Drop lateral/rotation toward zero too when fully blocked.
    r.command.vy = 0.0;
    r.command.w  = 0.0;
    r.state = (std::fabs(nv) < 1e-3) ? ControllerState::STUCK
                                     : ControllerState::TRACKING;
    return r;
  }

  // Scale the command magnitude down to v_safe, preserving direction.
  // Brake feasibility (v_safe) is computed from acc_lim_x / vx only: vy is scaled by
  // the same factor and w is passed through. The swept check still rotates the
  // footprint by the predicted yaw, so a rotation or strafe that would put a center on
  // lethal is caught and braked; but the "can always brake before contact" margin is
  // rigorous for forward motion. This suits the forward-facing motion policy
  // (strafe/rotate are secondary); revisit if lateral/rotational speed dominates.
  const double scale = v_safe / std::fabs(vx);
  r.command.vx = vx * scale;
  r.command.vy = mpc.applied.vy * scale;
  r.command.w  = mpc.applied.w;   // keep yaw authority for re-aiming
  r.state = ControllerState::TRACKING;
  return r;
}

}  // namespace g1_local_planner
