// foot_ground unit tests: the kinematic floor estimate projects the foot sole
// collision-sphere centres through the foot pose and returns the lowest map Z
// minus the sphere radius. Numbers verified against the g1_description ankle_roll
// collision (centres z=-0.03, r=0.005, toe x=0.12, heel x=-0.05).

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Geometry>

#include "g1_costmap/core/foot_ground.h"

using namespace g1_costmap;

namespace {
// The 4 sole-sphere centres in the ankle_roll frame (identical for both feet).
std::vector<Eigen::Vector3d> centres() {
  return {{0.12, 0.03, -0.03},
          {0.12, -0.03, -0.03},
          {-0.05, 0.025, -0.03},
          {-0.05, -0.025, -0.03}};
}
constexpr double kR = 0.005;
const Eigen::Vector3d kY = Eigen::Vector3d::UnitY();
const Eigen::Vector3d kX = Eigen::Vector3d::UnitX();
const Eigen::Isometry3d kNone = Eigen::Isometry3d::Identity();

// A foot pose: origin at height H, rotated by `angle` (rad) about `axis`.
Eigen::Isometry3d foot(double H, double angle, const Eigen::Vector3d& axis) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(0.0, 0.0, H);
  T.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
  return T;
}
double deg(double d) { return d * M_PI / 180.0; }
}  // namespace

TEST(FootGround, FlatFootIsOriginMinusSole) {
  // Flat foot at H: sole bottom = H - 0.03 - 0.005 = H - 0.035 (parity with the
  // old fixed scalar on flat ground).
  const double H = 1.0;
  EXPECT_NEAR(
      footGroundFromContacts(foot(H, 0, kY), true, kNone, false, centres(), kR),
      H - 0.035, 1e-9);
}

TEST(FootGround, PitchToeDownLowersGround) {
  // +30 deg about Y = toe down: the toe sphere is lowest -> H - 0.090981.
  const double H = 1.0;
  EXPECT_NEAR(footGroundFromContacts(foot(H, deg(30), kY), true, kNone, false,
                                     centres(), kR),
              H - 0.090981, 1e-6);
}

TEST(FootGround, PitchToeUpUsesHeel) {
  // -30 deg about Y = toe up: the heel sphere is lowest -> H - 0.055981.
  const double H = 1.0;
  EXPECT_NEAR(footGroundFromContacts(foot(H, deg(-30), kY), true, kNone, false,
                                     centres(), kR),
              H - 0.055981, 1e-6);
}

TEST(FootGround, RollIsSmallButExact) {
  // +15 deg about X -> H - 0.041742.
  const double H = 1.0;
  EXPECT_NEAR(footGroundFromContacts(foot(H, deg(15), kX), true, kNone, false,
                                     centres(), kR),
              H - 0.041742, 1e-6);
}

TEST(FootGround, SingleFootIgnoresUnresolved) {
  // Only the right foot resolves; the (garbage) left pose must be ignored.
  const double H = 0.5;
  const Eigen::Isometry3d garbage = foot(-100.0, 0, kY);
  EXPECT_NEAR(footGroundFromContacts(garbage, false, foot(H, 0, kY), true,
                                     centres(), kR),
              H - 0.035, 1e-9);
}

TEST(FootGround, TwoFeetTakeLowerSole) {
  // Left foot lower than right -> the left sole defines the ground.
  const double Hl = 0.40, Hr = 0.55;
  EXPECT_NEAR(footGroundFromContacts(foot(Hl, 0, kY), true, foot(Hr, 0, kY),
                                     true, centres(), kR),
              Hl - 0.035, 1e-9);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
