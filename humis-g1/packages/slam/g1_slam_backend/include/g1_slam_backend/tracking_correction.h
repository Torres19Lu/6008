#pragma once

#include <Eigen/Geometry>

namespace g1_slam_backend {

// Tuning for the localization-tracking robust correction. All thresholds have a
// disable sentinel so the behavior degrades gracefully to "apply the raw ICP
// result" (the legacy behavior) when everything is disabled.
struct TrackingCorrectionConfig {
  // Degeneracy projection (LIO-SAM style). A registration-Hessian eigenvalue is
  // "observable" iff it is >= obs_eig_floor AND >= obs_eig_ratio * max_eig; the
  // correction component along any other (degenerate) eigenvector is dropped so
  // the pose does not slide along an unconstrained direction. <=0 disables that
  // test. The info matrix passed in is normalized (mean over scan points), so the
  // translation sub-block eigenvalues lie in [0, 1].
  double obs_eig_floor = 0.0;
  double obs_eig_ratio = 0.0;
  // Low-pass: fraction of the (projected, clamped) correction applied per tick.
  // 1.0 == apply it all (no smoothing); the front-end odom carries motion between
  // ticks, so a fraction lets map->odom converge over a few ticks without jitter.
  double corr_gain = 1.0;
  // Per-tick step clamp on the correction (after projection). <=0 disables.
  double max_step_trans = 0.0;  // m
  double max_step_rot = 0.0;    // rad
  // Reject (coast on odom) if the RAW correction exceeds this. <=0 disables.
  double reject_trans = 0.0;    // m
  double reject_rot = 0.0;      // rad
};

struct TrackingCorrectionResult {
  Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
  bool applied = false;       // a (possibly zero) correction was written
  bool rejected = false;      // raw jump too large / no observable DOF -> coast
  int num_degenerate = 0;     // count of suppressed (unobservable) DOFs
};

// Turn a raw scan-to-map ICP pose into a safe map->odom correction.
//   map_to_odom_cur : the current correction (coasted on reject)
//   odom_pose       : the live front-end odom pose T(odom<-base) at this scan
//   x_now_raw       : the raw ICP result T(map<-base_now)
//   obs_info        : the 6x6 registration information matrix at the converged
//                     pose, ordered [trans(3); rot(3)] in the map frame about the
//                     sensor position (mean-normalized; see TrackingCorrectionConfig)
// The predicted pose is pred = map_to_odom_cur * odom_pose; the correction is the
// delta from pred to x_now_raw, which is rejected / projected / clamped / blended
// per cfg before being folded back into map->odom.
TrackingCorrectionResult computeTrackingCorrection(const Eigen::Isometry3d& map_to_odom_cur,
                                        const Eigen::Isometry3d& odom_pose,
                                        const Eigen::Isometry3d& x_now_raw,
                                        const Eigen::Matrix<double, 6, 6>& obs_info,
                                        const TrackingCorrectionConfig& cfg);

}  // namespace g1_slam_backend
