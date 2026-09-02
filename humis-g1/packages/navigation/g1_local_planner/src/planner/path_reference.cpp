// path_reference.cpp -- arc-length rolling reference implementation.
// See path_reference.h for the full specification and design notes.
// No ROS headers. No Eigen. Pure std math.

#include "g1_local_planner/planner/path_reference.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace g1_local_planner {

// ---------------------------------------------------------------------------
// Internal helpers

// Squared distance between two points.
static double sqDist(double ax, double ay, double bx, double by) {
  const double dx = ax - bx;
  const double dy = ay - by;
  return dx * dx + dy * dy;
}

// Arc length of the full polyline.
static double totalArcLength(const std::vector<Pose2D>& path) {
  double total = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total;
}

// ---------------------------------------------------------------------------
// projectOntoPath

PathProjection projectOntoPath(const std::vector<Pose2D>& path,
                               double x, double y) {
  PathProjection result;

  if (path.empty()) {
    result.valid = false;
    return result;
  }

  result.valid = true;

  // Single-point degenerate case.
  if (path.size() == 1) {
    result.arc_length  = 0.0;
    result.point.x     = path[0].x;
    result.point.y     = path[0].y;
    result.tangent_yaw = 0.0;
    result.seg         = 0;
    return result;
  }

  double best_dist_sq    = std::numeric_limits<double>::max();
  double arc_to_best     = 0.0;
  double best_px         = path[0].x;
  double best_py         = path[0].y;
  double best_tan_yaw    = 0.0;
  size_t best_seg        = 0;

  double arc_accum = 0.0;  // arc length from path[0] to path[i]

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const double ax = path[i].x;
    const double ay = path[i].y;
    const double bx = path[i + 1].x;
    const double by = path[i + 1].y;
    const double sdx = bx - ax;
    const double sdy = by - ay;
    const double seg_len_sq = sdx * sdx + sdy * sdy;

    double t = 0.0;
    double px = ax;
    double py = ay;
    double tan_yaw = std::atan2(sdy, sdx);

    if (seg_len_sq > 0.0) {
      // Clamped projection parameter along the segment.
      t = ((x - ax) * sdx + (y - ay) * sdy) / seg_len_sq;
      t = std::max(0.0, std::min(1.0, t));
      px = ax + t * sdx;
      py = ay + t * sdy;
    }

    const double d_sq = sqDist(x, y, px, py);
    if (d_sq < best_dist_sq) {
      best_dist_sq  = d_sq;
      best_seg      = i;
      best_px       = px;
      best_py       = py;
      best_tan_yaw  = tan_yaw;
      // Arc length from path[0] to projection = arc to segment start + t * segment len.
      const double seg_len = std::sqrt(seg_len_sq);
      arc_to_best = arc_accum + t * seg_len;
    }

    // Accumulate arc length past this segment (for next iteration).
    arc_accum += std::sqrt(seg_len_sq);
  }

  result.arc_length  = arc_to_best;
  result.point.x     = best_px;
  result.point.y     = best_py;
  result.tangent_yaw = best_tan_yaw;
  result.seg         = best_seg;
  return result;
}

// ---------------------------------------------------------------------------
// pointAtArcLength

Pose2D pointAtArcLength(const std::vector<Pose2D>& path, double s,
                        double& out_tangent_yaw) {
  Pose2D result;

  // Single-point degenerate case.
  if (path.empty()) {
    out_tangent_yaw = 0.0;
    return result;
  }

  if (path.size() == 1) {
    result.x       = path[0].x;
    result.y       = path[0].y;
    out_tangent_yaw = 0.0;
    return result;
  }

  // Clamp s to [0, total].
  if (s <= 0.0) {
    result.x        = path[0].x;
    result.y        = path[0].y;
    const double dx = path[1].x - path[0].x;
    const double dy = path[1].y - path[0].y;
    out_tangent_yaw = std::atan2(dy, dx);
    return result;
  }

  double arc_accum = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx      = path[i + 1].x - path[i].x;
    const double dy      = path[i + 1].y - path[i].y;
    const double seg_len = std::sqrt(dx * dx + dy * dy);
    const double arc_end = arc_accum + seg_len;

    if (s <= arc_end || i + 2 == path.size()) {
      // s falls within this segment (or s > total: clamp at last segment end).
      double t = 0.0;
      if (seg_len > 0.0) {
        t = (s - arc_accum) / seg_len;
        t = std::max(0.0, std::min(1.0, t));  // clamp handles s > total
      }
      result.x        = path[i].x + t * dx;
      result.y        = path[i].y + t * dy;
      out_tangent_yaw = (seg_len > 0.0) ? std::atan2(dy, dx) : 0.0;
      return result;
    }

    arc_accum = arc_end;
  }

  // Unreachable for non-empty path, but guard anyway.
  result.x        = path.back().x;
  result.y        = path.back().y;
  const size_t n  = path.size();
  const double dx = path[n - 1].x - path[n - 2].x;
  const double dy = path[n - 1].y - path[n - 2].y;
  out_tangent_yaw = std::atan2(dy, dx);
  return result;
}

// ---------------------------------------------------------------------------
// referenceSpeed

double referenceSpeed(const std::vector<Pose2D>& path,
                      const PathProjection& proj,
                      double total_len,
                      const Pose2D& robot,
                      const MpcConfig& cfg) {
  // Approach: decel-to-stop ramp.
  const double remaining  = std::max(0.0, total_len - proj.arc_length);
  const double v_approach = std::sqrt(2.0 * std::max(0.0, cfg.a_decel) * remaining);

  // Curvature: scan the upcoming window for the max |dtheta/ds| and cap lateral
  // accel. Sampling tangents over sub-intervals concentrates a sharp bend into
  // one interval; a finely sampled smooth arc converges to kappa = 1/R.
  constexpr double kKappaEps  = 1e-3;  // 1/m; floor so straight runs do not bind
  constexpr int    kCurvSteps = 5;     // sub-intervals across the window
  double max_kappa = 0.0;
  const double window = std::min(cfg.curv_lookahead, remaining);
  if (window > 0.0) {
    const double ds = window / static_cast<double>(kCurvSteps);
    double prev_yaw = 0.0;
    pointAtArcLength(path, proj.arc_length, prev_yaw);
    for (int i = 1; i <= kCurvSteps; ++i) {
      double yaw_i = 0.0;
      pointAtArcLength(path, proj.arc_length + ds * static_cast<double>(i), yaw_i);
      const double dtheta = std::abs(wrapToPi(yaw_i - prev_yaw));
      const double kappa  = (ds > 0.0) ? dtheta / ds : 0.0;
      if (kappa > max_kappa) max_kappa = kappa;
      prev_yaw = yaw_i;
    }
  }
  const double v_curv =
      std::sqrt(std::max(0.0, cfg.a_lat_max) / std::max(max_kappa, kKappaEps));

  // Heading error: slow translation when the robot must turn to face the travel
  // direction (covers sharp turns and large-angle reversals). Keyed on the path
  // tangent, NOT the goal yaw (the approach ramp + MPC terminal yaw handle final
  // alignment), so the goal-align reference behavior is unchanged.
  const double theta_err = std::abs(wrapToPi(proj.tangent_yaw - robot.yaw));
  const double span = std::max(1e-6, cfg.heading_slow_full - cfg.heading_slow_start);
  double factor = (cfg.heading_slow_full - theta_err) / span;
  factor = std::max(cfg.heading_slow_floor, std::min(1.0, factor));
  const double v_heading = cfg.v_ref * factor;

  // Cruise regulators floored at v_min_move; the approach ramp can override to 0.
  const double v_cruise =
      std::max(std::min({cfg.v_ref, v_curv, v_heading}), cfg.v_min_move);

  return std::min(v_cruise, v_approach);
}

// ---------------------------------------------------------------------------
// generateReference

std::vector<Pose2D> generateReference(const std::vector<Pose2D>& path,
                                      const Pose2D& robot,
                                      const Pose2D& goal,
                                      const MpcConfig& cfg) {
  if (path.empty()) {
    return {};
  }

  const PathProjection proj = projectOntoPath(path, robot.x, robot.y);
  const double total_len    = totalArcLength(path);
  const double remaining    = total_len - proj.arc_length;
  const bool   goal_align   = (remaining <= cfg.goal_align_radius);
  // A reachable goal (path endpoint within reach_goal_gap of the raw goal) lets the
  // clamped terminal refs pile up at the RAW goal so the MPC decelerates into the true
  // goal, not the grid-snapped endpoint. Unreachable -> clamp at the endpoint as before.
  const bool   goal_reachable =
      goal_align &&
      (std::hypot(path.back().x - goal.x, path.back().y - goal.y) <= cfg.reach_goal_gap);

  const double v_target = referenceSpeed(path, proj, total_len, robot, cfg);

  std::vector<Pose2D> refs;
  refs.reserve(static_cast<size_t>(cfg.horizon));

  for (int k = 1; k <= cfg.horizon; ++k) {
    const double raw_s = proj.arc_length + v_target * cfg.dt * static_cast<double>(k);
    const double s_k   = std::min(raw_s, total_len);

    double tan_yaw = 0.0;
    Pose2D p       = pointAtArcLength(path, s_k, tan_yaw);

    Pose2D ref;
    ref.x   = p.x;
    ref.y   = p.y;
    ref.yaw = goal_align ? goal.yaw : tan_yaw;

    // Terminal refs (clamped at the path end) retarget to the raw goal when reachable.
    if (goal_reachable && raw_s >= total_len) {
      ref.x = goal.x;
      ref.y = goal.y;
    }

    refs.push_back(ref);
  }

  return refs;
}

}  // namespace g1_local_planner
