#include "g1_slam_backend/tracking_correction.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

namespace g1_slam_backend {
namespace {

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;

// Axis-angle (rotation vector) of a small rotation matrix.
Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
  const Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

// Rotation matrix from a rotation vector (Identity for a ~zero vector).
Eigen::Matrix3d expSO3(const Eigen::Vector3d& w) {
  const double a = w.norm();
  if (a < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(a, w / a).toRotationMatrix();
}

}  // namespace

TrackingCorrectionResult computeTrackingCorrection(const Eigen::Isometry3d& map_to_odom_cur,
                                        const Eigen::Isometry3d& odom_pose,
                                        const Eigen::Isometry3d& x_now_raw,
                                        const Mat6& obs_info,
                                        const TrackingCorrectionConfig& cfg) {
  TrackingCorrectionResult res;
  res.map_to_odom = map_to_odom_cur;  // default: coast

  // Predicted sensor map pose from the smooth front-end odom + current correction.
  const Eigen::Isometry3d pred = map_to_odom_cur * odom_pose;

  // Raw correction the ICP wants, pred -> x_now_raw (map frame, about the sensor).
  const Eigen::Vector3d d_trans = x_now_raw.translation() - pred.translation();
  const Eigen::Vector3d d_rot = logSO3(x_now_raw.linear() * pred.linear().transpose());

  // Reject an implausibly large RAW jump: coast on odom (the front end carries
  // motion; a 1.5 m teleport while the robot barely moved is a bad registration).
  if ((cfg.reject_trans > 0.0 && d_trans.norm() > cfg.reject_trans) ||
      (cfg.reject_rot > 0.0 && d_rot.norm() > cfg.reject_rot)) {
    res.rejected = true;
    return res;
  }

  Vec6 delta;
  delta << d_trans, d_rot;

  // Degeneracy projection (LIO-SAM style): drop the correction component along
  // any unobservable eigenvector of the registration Hessian, so the pose cannot
  // slide along an unconstrained direction (e.g. across a dominant table plane).
  Vec6 delta_proj = delta;
  const bool project = (cfg.obs_eig_floor > 0.0 || cfg.obs_eig_ratio > 0.0);
  if (project) {
    Eigen::SelfAdjointEigenSolver<Mat6> es(obs_info);
    const Vec6 eig = es.eigenvalues();          // ascending
    const Mat6 V = es.eigenvectors();           // columns
    const double max_eig = eig.maxCoeff();
    const double thr =
        std::max(cfg.obs_eig_floor, cfg.obs_eig_ratio * std::max(max_eig, 0.0));
    delta_proj.setZero();
    int observable = 0;
    for (int i = 0; i < 6; ++i) {
      if (eig(i) >= thr) {                       // observable DOF -> keep
        const Vec6 v = V.col(i);
        delta_proj += (v.dot(delta)) * v;
        ++observable;
      } else {
        ++res.num_degenerate;
      }
    }
    if (observable == 0) {                        // no information at all -> coast
      res.rejected = true;
      res.num_degenerate = 6;
      return res;
    }
  }

  Eigen::Vector3d t_step = delta_proj.head<3>();
  Eigen::Vector3d r_step = delta_proj.tail<3>();

  // Clamp the per-tick step (after projection) so a single update stays bounded.
  if (cfg.max_step_trans > 0.0 && t_step.norm() > cfg.max_step_trans) {
    t_step *= cfg.max_step_trans / t_step.norm();
  }
  if (cfg.max_step_rot > 0.0 && r_step.norm() > cfg.max_step_rot) {
    r_step *= cfg.max_step_rot / r_step.norm();
  }

  // Low-pass: apply a fraction of the (projected, clamped) correction per tick.
  const double gain = std::min(1.0, std::max(0.0, cfg.corr_gain));
  t_step *= gain;
  r_step *= gain;

  // Fold the correction back into map->odom via the corrected sensor pose.
  Eigen::Isometry3d x_corr = Eigen::Isometry3d::Identity();
  x_corr.linear() = expSO3(r_step) * pred.linear();
  x_corr.translation() = pred.translation() + t_step;

  res.map_to_odom = x_corr * odom_pose.inverse();
  res.applied = true;
  return res;
}

}  // namespace g1_slam_backend
