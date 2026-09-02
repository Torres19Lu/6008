#pragma once

// path_reference.h -- ROS-free arc-length rolling reference for g1_local_planner.
// No ROS headers. No Eigen. Pure std math on std::vector<Pose2D>.
//
// Implements an arc-length moving reference (yaw_ref = path tangent so the robot
// faces its travel direction). Within goal_align_radius, yaw_ref switches to
// goal.yaw and reference points clamp at the path end, so the MPC naturally
// decelerates into the goal.
//
// Key design notes:
//   - yaw values are NOT normalised here; consumers normalise the yaw ERROR when
//     comparing (e.g. in the MPC cost). This keeps the reference layer simple and
//     free of wrapping logic.
//   - clamping s_k at total_len makes reference points pile up at the goal as the
//     robot approaches, so the MPC position error naturally shrinks and the robot
//     decelerates without a separate terminal P-controller.
//   - The goal-align heading switch (remaining <= goal_align_radius) is evaluated
//     at the time generateReference is called, using remaining = total_len -
//     proj.arc_length. When remaining <= goal_align_radius all N ref yaws are
//     overridden with goal.yaw regardless of the tangent.

#include <cstddef>
#include <vector>

#include "g1_local_planner/core/plan_types.h"

namespace g1_local_planner {

// Result of projecting a point (x, y) onto the closest point on a polyline.
struct PathProjection {
  double arc_length  = 0.0;  // arc length from path[0] to the projected point (metres)
  Pose2D point;              // the closest point on the polyline (x, y; yaw unused)
  double tangent_yaw = 0.0;  // atan2(dy, dx) of the segment that owns this projection
  size_t seg         = 0;    // index i such that the projection lies on [path[i], path[i+1]]
  bool   valid       = false; // false only when path is empty
};

// Project (x, y) onto the closest point of the polyline defined by path.
// For each segment [i, i+1] computes the clamped projection parameter t in [0,1],
// the closest point, and the perpendicular distance; selects the segment with the
// minimum distance.
// Edge cases:
//   path.empty()       -> PathProjection with valid=false
//   path.size() == 1   -> projects to that single point; arc_length=0, tangent_yaw=0, seg=0
PathProjection projectOntoPath(const std::vector<Pose2D>& path, double x, double y);

// Return the Pose2D (x, y; yaw=tangent) at arc length s along the polyline.
// Also writes the tangent yaw at that point into out_tangent_yaw.
// Clamp behaviour:
//   s <= 0              -> first point, tangent of the first segment (or 0 if single point)
//   s >= total_length   -> last point,  tangent of the last  segment (or 0 if single point)
//   0 < s < total       -> linearly interpolated between the two endpoints of the spanning segment
// For a single-point path the returned pose is that point with out_tangent_yaw=0.
Pose2D pointAtArcLength(const std::vector<Pose2D>& path, double s,
                        double& out_tangent_yaw);

// Generate N reference states (Pose2D with x, y, yaw_ref) for use by the MPC.
// Parameters:
//   path  -- global path polyline (map frame, from /nav/global_path)
//   robot -- current robot pose (map frame, from TF)
//   goal  -- the navigation goal pose (used for yaw in the goal-align zone)
//   cfg   -- MpcConfig supplying horizon, dt, v_ref, goal_align_radius
// Returns:
//   vector of cfg.horizon Pose2D entries: ref[k] = reference state for step k+1.
//   ref[k].x, ref[k].y = position on (or clamped to the end of) the path.
//   ref[k].yaw = path tangent, except within goal_align_radius where it equals goal.yaw.
//   Returns an empty vector when path is empty (the LocalPlanner facade maps this
//   to the NO_PATH state and issues a zero command).
// Algorithm:
//   proj = projectOntoPath(path, robot.x, robot.y)
//   total_len = sum of all segment lengths
//   remaining = total_len - proj.arc_length
//   for k = 1..cfg.horizon:
//     v_target = referenceSpeed(path, proj, total_len, robot, cfg)
//     s_k = clamp(proj.arc_length + v_target * cfg.dt * k, 0, total_len)
//     p, tan_yaw = pointAtArcLength(path, s_k)
//     ref.yaw = (remaining <= cfg.goal_align_radius) ? goal.yaw : tan_yaw
std::vector<Pose2D> generateReference(const std::vector<Pose2D>& path,
                                      const Pose2D& robot,
                                      const Pose2D& goal,
                                      const MpcConfig& cfg);

// Per-cycle target speed (m/s, >= 0) for the arc-length reference, the minimum of:
//   - approach ramp  v_approach = sqrt(2 * a_decel * remaining)  (decel to stop)
//   - curvature cap  v_curv     = sqrt(a_lat_max / |kappa|)      (slow in turns)
//   - heading factor v_heading  = v_ref * f(|tangent - robot.yaw|) (turn-in-place)
// The cruise regulators (v_ref, v_curv, v_heading) are floored at v_min_move;
// v_approach is applied AFTER the floor so the speed can reach 0 at the goal.
// proj/total_len are the projection and total arc length already computed by
// generateReference (passed in to avoid recomputation).
double referenceSpeed(const std::vector<Pose2D>& path,
                      const PathProjection& proj,
                      double total_len,
                      const Pose2D& robot,
                      const MpcConfig& cfg);

}  // namespace g1_local_planner
