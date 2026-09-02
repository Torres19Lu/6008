#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "g1_description/base_footprint.hpp"

using g1_description::computeBaseToFootprint;

namespace {
Eigen::Isometry3d makePose(double x, double y, double z,
                           double roll, double pitch, double yaw) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  t.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return t;
}
}  // namespace

// Upright pelvis 0.8 m up at the origin: footprint is straight down by 0.8,
// no rotation relative to base.
TEST(BaseFootprint, UprightDropsToGround) {
  Eigen::Isometry3d odom_T_base = makePose(1.0, 2.0, 0.8, 0, 0, 0);
  Eigen::Isometry3d base_T_foot = computeBaseToFootprint(odom_T_base);
  EXPECT_NEAR(base_T_foot.translation().x(), 0.0, 1e-9);
  EXPECT_NEAR(base_T_foot.translation().y(), 0.0, 1e-9);
  EXPECT_NEAR(base_T_foot.translation().z(), -0.8, 1e-9);
  EXPECT_TRUE(base_T_foot.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

// Pure yaw does not tilt base, so footprint stays directly below with identity
// relative rotation (footprint yaw equals base yaw).
TEST(BaseFootprint, PureYawStaysIdentityRelative) {
  Eigen::Isometry3d odom_T_base = makePose(1.0, 2.0, 0.8, 0, 0, 1.2);
  Eigen::Isometry3d base_T_foot = computeBaseToFootprint(odom_T_base);
  EXPECT_NEAR(base_T_foot.translation().z(), -0.8, 1e-9);
  EXPECT_TRUE(base_T_foot.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
}

// Invariant: for any base pose, the resulting footprint in odom is gravity
// aligned (zero roll/pitch) and sits on the ground (z = 0).
TEST(BaseFootprint, FootprintIsGravityAlignedOnGround) {
  Eigen::Isometry3d odom_T_base = makePose(0.5, -0.3, 0.79, 0.05, 0.10, 0.7);
  Eigen::Isometry3d odom_T_foot = odom_T_base * computeBaseToFootprint(odom_T_base);
  EXPECT_NEAR(odom_T_foot.translation().z(), 0.0, 1e-9);
  // Z axis of the footprint must point straight up.
  Eigen::Vector3d z_axis = odom_T_foot.linear().col(2);
  EXPECT_TRUE(z_axis.isApprox(Eigen::Vector3d::UnitZ(), 1e-9));
  // X/Y stay under the base.
  EXPECT_NEAR(odom_T_foot.translation().x(), 0.5, 1e-9);
  EXPECT_NEAR(odom_T_foot.translation().y(), -0.3, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
