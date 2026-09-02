// Core pose-math round-trip + SE(3) sanity for the g1_map_manager types.

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "g1_map_manager/core/map_types.h"

using namespace g1_map_manager;

TEST(PoseMath, ToIsoFromIsoRoundTrip) {
  Pose3 p;
  p.x = 1.5;
  p.y = -2.0;
  p.z = 0.3;
  Eigen::Quaterniond q(
      Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 0.5, 1.0).normalized()));
  q.normalize();
  p.qx = q.x();
  p.qy = q.y();
  p.qz = q.z();
  p.qw = q.w();

  const Eigen::Isometry3d T = toIso(p);
  const Pose3 r = fromIso(T);
  EXPECT_NEAR(r.x, p.x, 1e-12);
  EXPECT_NEAR(r.y, p.y, 1e-12);
  EXPECT_NEAR(r.z, p.z, 1e-12);
  // Quaternion sign may flip; compare the rotation matrices instead.
  EXPECT_TRUE(toIso(r).rotation().isApprox(T.rotation(), 1e-9));
}

TEST(PoseMath, InverseComposesToIdentity) {
  Pose3 p;
  p.x = 3.0;
  p.y = 4.0;
  p.z = -1.0;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()));
  p.qx = q.x();
  p.qy = q.y();
  p.qz = q.z();
  p.qw = q.w();
  const Eigen::Isometry3d T = toIso(p);
  EXPECT_TRUE((T.inverse() * T).isApprox(Eigen::Isometry3d::Identity(), 1e-12));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
