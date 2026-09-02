#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <g1_msgs/LocoStatus.h>

#include "g1_locomotion/loco_safety.hpp"

using namespace g1_locomotion;

TEST(Clamp, WithinLimitsUnchanged) {
  VelocityLimits lim{1.0, 1.0, 1.0};
  Velocity v = clampVelocity(Velocity{0.5, -0.5, 0.2}, lim);
  EXPECT_DOUBLE_EQ(v.vx, 0.5);
  EXPECT_DOUBLE_EQ(v.vy, -0.5);
  EXPECT_DOUBLE_EQ(v.vyaw, 0.2);
}

TEST(Clamp, OverLimitsSaturates) {
  VelocityLimits lim{0.3, 0.2, 0.4};
  Velocity v = clampVelocity(Velocity{1.0, -1.0, 5.0}, lim);
  EXPECT_DOUBLE_EQ(v.vx, 0.3);
  EXPECT_DOUBLE_EQ(v.vy, -0.2);
  EXPECT_DOUBLE_EQ(v.vyaw, 0.4);
}

TEST(Clamp, NegativeLimitTreatedAsZero) {
  VelocityLimits lim{-1.0, 0.0, 0.0};
  Velocity v = clampVelocity(Velocity{0.5, 0.5, 0.5}, lim);
  EXPECT_DOUBLE_EQ(v.vx, 0.0);
  EXPECT_DOUBLE_EQ(v.vy, 0.0);
  EXPECT_DOUBLE_EQ(v.vyaw, 0.0);
}

TEST(Clamp, NonFiniteBecomesZero) {
  VelocityLimits lim{1.0, 1.0, 1.0};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  Velocity v = clampVelocity(Velocity{nan, inf, -inf}, lim);
  EXPECT_DOUBLE_EQ(v.vx, 0.0);
  EXPECT_DOUBLE_EQ(v.vy, 0.0);
  EXPECT_DOUBLE_EQ(v.vyaw, 0.0);
}

TEST(Clamp, AsymmetricVxForwardBackward) {
  VelocityLimits lim;
  lim.vx_max = 0.5;       // forward cap
  lim.vx_back_max = 0.3;  // backward cap
  lim.vy_max = 0.3;
  lim.vyaw_max = 0.8;
  EXPECT_DOUBLE_EQ(clampVelocity(Velocity{0.9, 0.0, 0.0}, lim).vx, 0.5);
  EXPECT_DOUBLE_EQ(clampVelocity(Velocity{-0.9, 0.0, 0.0}, lim).vx, -0.3);
  EXPECT_DOUBLE_EQ(clampVelocity(Velocity{0.2, 0.0, 0.0}, lim).vx, 0.2);
  EXPECT_DOUBLE_EQ(clampVelocity(Velocity{-0.2, 0.0, 0.0}, lim).vx, -0.2);
}

TEST(Clamp, ZeroBackMaxIsSymmetric) {
  VelocityLimits lim;  // vx_back_max defaults 0 -> symmetric (= vx_max)
  lim.vx_max = 0.3;
  lim.vy_max = 0.2;
  lim.vyaw_max = 0.4;
  EXPECT_DOUBLE_EQ(clampVelocity(Velocity{-0.9, 0.0, 0.0}, lim).vx, -0.3);
  EXPECT_DOUBLE_EQ(clampVelocity(Velocity{0.9, 0.0, 0.0}, lim).vx, 0.3);
}

TEST(Slew, StepsTowardTargetBounded) {
  AccelLimits a{1.0, 1.0, 1.0};  // dt=0.1 -> max step 0.1/axis
  Velocity v = slewVelocity(Velocity{}, Velocity{1.0, -1.0, 0.05}, a, 0.1);
  EXPECT_NEAR(v.vx, 0.1, 1e-9);
  EXPECT_NEAR(v.vy, -0.1, 1e-9);
  EXPECT_NEAR(v.vyaw, 0.05, 1e-9);  // within one step -> reaches target
}

TEST(Slew, ReachesTargetNoOvershoot) {
  AccelLimits a{10.0, 10.0, 10.0};
  Velocity v = slewVelocity(Velocity{}, Velocity{0.2, 0.0, 0.0}, a, 1.0);
  EXPECT_DOUBLE_EQ(v.vx, 0.2);
}

TEST(Slew, NonPositiveDtHolds) {
  AccelLimits a{1.0, 1.0, 1.0};
  Velocity v = slewVelocity(Velocity{0.3, 0.0, 0.0}, Velocity{}, a, 0.0);
  EXPECT_DOUBLE_EQ(v.vx, 0.3);
}

TEST(Watchdog, FreshNotStale) { EXPECT_FALSE(isCommandStale(0.1, 0.3)); }
TEST(Watchdog, OldIsStale) { EXPECT_TRUE(isCommandStale(0.4, 0.3)); }
TEST(Watchdog, NegativeAgeNotStale) { EXPECT_FALSE(isCommandStale(-1.0, 0.3)); }

TEST(FsmMap, KnownIds) {
  EXPECT_EQ(fsmIdToLocoMode(0), g1_msgs::LocoStatus::MODE_ZERO_TORQUE);
  EXPECT_EQ(fsmIdToLocoMode(1), g1_msgs::LocoStatus::MODE_DAMP);
  EXPECT_EQ(fsmIdToLocoMode(4), g1_msgs::LocoStatus::MODE_STAND);
  EXPECT_EQ(fsmIdToLocoMode(500), g1_msgs::LocoStatus::MODE_WALK);
}
TEST(FsmMap, UnknownId) {
  EXPECT_EQ(fsmIdToLocoMode(999), g1_msgs::LocoStatus::MODE_UNKNOWN);
}

TEST(Deadzone, BelowThresholdBecomesZero) {
  VelocityDeadzone dz{0.08, 0.08, 0.10};
  Velocity v = applyDeadzone(Velocity{0.05, -0.07, 0.05}, dz);
  EXPECT_DOUBLE_EQ(v.vx, 0.0);
  EXPECT_DOUBLE_EQ(v.vy, 0.0);
  EXPECT_DOUBLE_EQ(v.vyaw, 0.0);
}

TEST(Deadzone, AtOrAboveThresholdPassesThrough) {
  VelocityDeadzone dz{0.08, 0.08, 0.10};
  Velocity v = applyDeadzone(Velocity{0.20, -0.15, 0.30}, dz);
  EXPECT_DOUBLE_EQ(v.vx, 0.20);
  EXPECT_DOUBLE_EQ(v.vy, -0.15);
  EXPECT_DOUBLE_EQ(v.vyaw, 0.30);
}

TEST(Deadzone, BoundaryEqualsThresholdPassesThrough) {
  // |v| == deadzone is NOT below the threshold -> passes through unchanged.
  VelocityDeadzone dz{0.08, 0.08, 0.10};
  Velocity v = applyDeadzone(Velocity{0.08, -0.08, 0.10}, dz);
  EXPECT_DOUBLE_EQ(v.vx, 0.08);
  EXPECT_DOUBLE_EQ(v.vy, -0.08);
  EXPECT_DOUBLE_EQ(v.vyaw, 0.10);
}

TEST(Deadzone, ZeroDeadzoneIsDisabled) {
  VelocityDeadzone dz{0.0, 0.0, 0.0};
  Velocity v = applyDeadzone(Velocity{0.001, -0.001, 0.001}, dz);
  EXPECT_DOUBLE_EQ(v.vx, 0.001);
  EXPECT_DOUBLE_EQ(v.vy, -0.001);
  EXPECT_DOUBLE_EQ(v.vyaw, 0.001);
}

TEST(Deadzone, NonFiniteBecomesZero) {
  VelocityDeadzone dz{0.08, 0.08, 0.10};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  Velocity v = applyDeadzone(Velocity{nan, inf, -inf}, dz);
  EXPECT_DOUBLE_EQ(v.vx, 0.0);
  EXPECT_DOUBLE_EQ(v.vy, 0.0);
  EXPECT_DOUBLE_EQ(v.vyaw, 0.0);
}

TEST(FsmAllowed, InWhitelist) {
  EXPECT_TRUE(fsmAllowed(500, {500, 801}));
  EXPECT_TRUE(fsmAllowed(801, {500, 801}));
}
TEST(FsmAllowed, NotInWhitelist) {
  EXPECT_FALSE(fsmAllowed(0, {500, 801}));
  EXPECT_FALSE(fsmAllowed(4, {500, 801}));
  EXPECT_FALSE(fsmAllowed(-1, {500, 801}));
}
TEST(FsmAllowed, EmptyWhitelist) { EXPECT_FALSE(fsmAllowed(500, {})); }

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
