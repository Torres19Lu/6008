// Unit tests for the localization-tracking robust correction (ROS-free core).
//
// In LOCALIZATION mode the backend must NOT overwrite map->odom with the raw
// per-tick scan-to-map ICP result: near a planar surface (a table/monitor) that
// ICP is geometrically under-constrained and swings, while the front-end odom is
// smooth and trustworthy. computeTrackingCorrection() turns the raw ICP pose into a
// safe correction by (1) rejecting an implausibly large raw jump (coast on
// odom), (2) projecting the correction off the unobservable DOFs of the
// registration Hessian (LIO-SAM-style degeneracy handling), (3) clamping the
// per-tick step, and (4) blending it in with a gain (low-pass).
//
// Convention: the 6x6 observability info is ordered [trans(3); rot(3)] in the
// map frame about the sensor position; the correction delta uses the same order.

#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Geometry>

#include "g1_slam_backend/tracking_correction.h"

using g1_slam_backend::TrackingCorrectionConfig;
using g1_slam_backend::TrackingCorrectionResult;
using g1_slam_backend::computeTrackingCorrection;

namespace {

Eigen::Isometry3d xyzyaw(double x, double y, double z, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() << x, y, z;
  return T;
}

// All-DOF-observable info (well-conditioned scene): identity * scale.
Eigen::Matrix<double, 6, 6> isotropicInfo(double scale) {
  return Eigen::Matrix<double, 6, 6>::Identity() * scale;
}

// Default config: gain 1, generous clamps/reject, projection enabled.
TrackingCorrectionConfig baseCfg() {
  TrackingCorrectionConfig c;
  c.obs_eig_floor = 0.5;
  c.obs_eig_ratio = 0.0;   // ratio disabled by default in tests
  c.corr_gain = 1.0;
  c.max_step_trans = 1e9;
  c.max_step_rot = 1e9;
  c.reject_trans = 1e9;
  c.reject_rot = 1e9;
  return c;
}

// Sensor map pose recovered from a correction result + the live odom pose.
Eigen::Isometry3d sensorPose(const TrackingCorrectionResult& r,
                             const Eigen::Isometry3d& odom) {
  return r.map_to_odom * odom;
}

constexpr double kEps = 1e-6;

}  // namespace

// 1. Well-conditioned + gain<1 -> the correction is low-passed (half applied).
TEST(TrackingCorrection, BlendsByGainWhenWellConditioned) {
  const Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d cur = Eigen::Isometry3d::Identity();  // pred == odom
  const Eigen::Isometry3d raw = xyzyaw(0.10, 0.0, 0.0, 0.0);    // ICP wants +0.10 x
  TrackingCorrectionConfig cfg = baseCfg();
  cfg.corr_gain = 0.5;

  const TrackingCorrectionResult r =
      computeTrackingCorrection(cur, odom, raw, isotropicInfo(1.0), cfg);

  EXPECT_TRUE(r.applied);
  EXPECT_FALSE(r.rejected);
  EXPECT_NEAR(sensorPose(r, odom).translation().x(), 0.05, 1e-3);  // half of 0.10
  EXPECT_NEAR(sensorPose(r, odom).translation().y(), 0.0, kEps);
}

// 2. A correction purely along an UNOBSERVABLE DOF is projected out (~0 motion),
//    while a correction along an observable DOF passes through.
TEST(TrackingCorrection, ProjectsOutDegenerateDirection) {
  const Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d cur = Eigen::Isometry3d::Identity();
  // Translation-x is unobservable (eigenvalue 0 < floor); the rest observable.
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity();
  info(0, 0) = 0.0;  // trans-x DOF degenerate
  TrackingCorrectionConfig cfg = baseCfg();

  // Correction entirely along the degenerate x -> suppressed.
  const TrackingCorrectionResult rx =
      computeTrackingCorrection(cur, odom, xyzyaw(0.30, 0.0, 0.0, 0.0), info, cfg);
  EXPECT_TRUE(rx.applied);
  EXPECT_GE(rx.num_degenerate, 1);
  EXPECT_NEAR(sensorPose(rx, odom).translation().x(), 0.0, 1e-3);

  // Correction entirely along the observable y -> passes through.
  const TrackingCorrectionResult ry =
      computeTrackingCorrection(cur, odom, xyzyaw(0.0, 0.30, 0.0, 0.0), info, cfg);
  EXPECT_NEAR(sensorPose(ry, odom).translation().y(), 0.30, 1e-3);
}

// 3. A large-but-not-rejected correction is clamped to the per-tick max step.
TEST(TrackingCorrection, ClampsPerTickStep) {
  const Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d cur = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d raw = xyzyaw(1.5, 0.0, 0.0, 0.0);
  TrackingCorrectionConfig cfg = baseCfg();
  cfg.max_step_trans = 0.10;
  cfg.reject_trans = 2.0;  // 1.5 < 2.0 -> clamp, not reject

  const TrackingCorrectionResult r =
      computeTrackingCorrection(cur, odom, raw, isotropicInfo(1.0), cfg);

  EXPECT_TRUE(r.applied);
  EXPECT_FALSE(r.rejected);
  EXPECT_NEAR(sensorPose(r, odom).translation().x(), 0.10, 1e-3);
}

// 4. An implausibly large raw jump is rejected: coast on odom (correction held).
TEST(TrackingCorrection, RejectsAndCoastsOnLargeJump) {
  const Eigen::Isometry3d odom = xyzyaw(2.0, 1.0, 0.0, 0.3);
  const Eigen::Isometry3d cur = xyzyaw(0.2, -0.1, 0.0, 0.05);  // existing correction
  const Eigen::Isometry3d pred = cur * odom;
  const Eigen::Isometry3d raw =
      xyzyaw(pred.translation().x() + 1.5, pred.translation().y(), 0.0, 0.3);
  TrackingCorrectionConfig cfg = baseCfg();
  cfg.reject_trans = 0.5;  // 1.5 jump > 0.5 -> reject

  const TrackingCorrectionResult r =
      computeTrackingCorrection(cur, odom, raw, isotropicInfo(1.0), cfg);

  EXPECT_TRUE(r.rejected);
  EXPECT_FALSE(r.applied);
  // map->odom unchanged (coast): identical to the prior correction.
  EXPECT_TRUE(r.map_to_odom.isApprox(cur, 1e-9));
}

// 5. A yaw correction in a well-conditioned scene is blended like translation.
TEST(TrackingCorrection, BlendsYawCorrection) {
  const Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d cur = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d raw = xyzyaw(0.0, 0.0, 0.0, 0.20);
  TrackingCorrectionConfig cfg = baseCfg();
  cfg.corr_gain = 0.5;

  const TrackingCorrectionResult r =
      computeTrackingCorrection(cur, odom, raw, isotropicInfo(1.0), cfg);

  const Eigen::Isometry3d X = sensorPose(r, odom);
  const double yaw = std::atan2(X.linear()(1, 0), X.linear()(0, 0));
  EXPECT_TRUE(r.applied);
  EXPECT_NEAR(yaw, 0.10, 1e-3);  // half of 0.20
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
