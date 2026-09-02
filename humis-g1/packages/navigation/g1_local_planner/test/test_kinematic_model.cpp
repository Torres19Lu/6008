// Unit tests for kinematic_model: controlMatrix, stepLinear, stepTrue,
// rolloutLinear.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "g1_local_planner/core/kinematic_model.h"
#include "g1_local_planner/core/plan_types.h"

using g1_local_planner::Pose2D;
using g1_local_planner::Twist2D;
using g1_local_planner::controlMatrix;
using g1_local_planner::rolloutLinear;
using g1_local_planner::stepLinear;
using g1_local_planner::stepTrue;

static constexpr double kTol = 1e-9;

// ---------------------------------------------------------------------------
// controlMatrix: shape and values.

// theta0=0, dt=0.1: B should equal diag(0.1, 0.1, 0.1).
TEST(ControlMatrix, ZeroHeadingIsScaledIdentity) {
  const double dt = 0.1;
  Eigen::Matrix3d B = controlMatrix(0.0, dt);

  // Diagonal entries.
  EXPECT_NEAR(B(0, 0), dt,  kTol);
  EXPECT_NEAR(B(1, 1), dt,  kTol);
  EXPECT_NEAR(B(2, 2), dt,  kTol);

  // All off-diagonal entries must be zero.
  EXPECT_NEAR(B(0, 1), 0.0, kTol);
  EXPECT_NEAR(B(0, 2), 0.0, kTol);
  EXPECT_NEAR(B(1, 0), 0.0, kTol);
  EXPECT_NEAR(B(1, 2), 0.0, kTol);
  EXPECT_NEAR(B(2, 0), 0.0, kTol);
  EXPECT_NEAR(B(2, 1), 0.0, kTol);
}

// theta0=pi/2, dt=0.1: a unit +vx command (u=[1,0,0]) should map to +y.
// B*[1,0,0]^T = [cos(pi/2)*dt, sin(pi/2)*dt, 0]^T = [0, dt, 0]^T.
TEST(ControlMatrix, HalfPiHeadingVxMapsToY) {
  const double dt = 0.1;
  const double theta0 = M_PI / 2.0;
  Eigen::Matrix3d B = controlMatrix(theta0, dt);

  Eigen::Vector3d u(1.0, 0.0, 0.0);
  Eigen::Vector3d delta = B * u;

  EXPECT_NEAR(delta(0), 0.0, kTol);   // no x displacement
  EXPECT_NEAR(delta(1), dt,  kTol);   // +y displacement
  EXPECT_NEAR(delta(2), 0.0, kTol);   // no yaw change
}

// theta0=pi/2, dt=0.1: a unit +vy command (u=[0,1,0]) should map to -x.
// B*[0,1,0]^T = [-sin(pi/2)*dt, cos(pi/2)*dt, 0]^T = [-dt, 0, 0]^T.
TEST(ControlMatrix, HalfPiHeadingVyMapsToNegX) {
  const double dt = 0.1;
  const double theta0 = M_PI / 2.0;
  Eigen::Matrix3d B = controlMatrix(theta0, dt);

  Eigen::Vector3d u(0.0, 1.0, 0.0);
  Eigen::Vector3d delta = B * u;

  EXPECT_NEAR(delta(0), -dt,  kTol);  // -x displacement
  EXPECT_NEAR(delta(1),  0.0, kTol);  // no y displacement
  EXPECT_NEAR(delta(2),  0.0, kTol);  // no yaw change
}

// ---------------------------------------------------------------------------
// stepLinear: consistency with controlMatrix.

// stepLinear must produce s + B(theta0,dt)*u, checked componentwise.
TEST(StepLinear, MatchesControlMatrix) {
  const double theta0 = 0.7;
  const double dt     = 0.1;

  Pose2D  s;  s.x = 1.0;  s.y = 2.0;  s.yaw = theta0;
  Twist2D u;  u.vx = 0.3;  u.vy = -0.1;  u.w = 0.5;

  Eigen::Matrix3d B = controlMatrix(theta0, dt);
  Eigen::Vector3d uv(u.vx, u.vy, u.w);
  Eigen::Vector3d delta = B * uv;

  Pose2D next = stepLinear(s, u, theta0, dt);

  EXPECT_NEAR(next.x,   s.x   + delta(0), kTol);
  EXPECT_NEAR(next.y,   s.y   + delta(1), kTol);
  EXPECT_NEAR(next.yaw, s.yaw + delta(2), kTol);
}

// theta0=0: +vx command advances +x by vx*dt; y and yaw unchanged.
TEST(StepLinear, ZeroHeadingVxAdvancesX) {
  const double dt  = 0.1;
  const double vx  = 0.5;
  Pose2D  s;   // default: x=y=yaw=0
  Twist2D u;   u.vx = vx;

  Pose2D next = stepLinear(s, u, 0.0, dt);

  EXPECT_NEAR(next.x,   vx * dt, kTol);
  EXPECT_NEAR(next.y,   0.0,     kTol);
  EXPECT_NEAR(next.yaw, 0.0,     kTol);
}

// theta0=0: +vy command advances +y by vy*dt.
TEST(StepLinear, ZeroHeadingVyAdvancesY) {
  const double dt = 0.1;
  const double vy = 0.3;
  Pose2D  s;
  Twist2D u;  u.vy = vy;

  Pose2D next = stepLinear(s, u, 0.0, dt);

  EXPECT_NEAR(next.x,   0.0,     kTol);
  EXPECT_NEAR(next.y,   vy * dt, kTol);
  EXPECT_NEAR(next.yaw, 0.0,     kTol);
}

// theta0=0: +w command advances yaw by w*dt; x and y unchanged.
TEST(StepLinear, ZeroHeadingWAdvancesYaw) {
  const double dt = 0.1;
  const double w  = 0.8;
  Pose2D  s;
  Twist2D u;  u.w = w;

  Pose2D next = stepLinear(s, u, 0.0, dt);

  EXPECT_NEAR(next.x,   0.0,    kTol);
  EXPECT_NEAR(next.y,   0.0,    kTol);
  EXPECT_NEAR(next.yaw, w * dt, kTol);
}

// ---------------------------------------------------------------------------
// stepTrue: nonlinear kinematics.

// s.yaw=pi/2: a +vx command should advance +y (not +x).
TEST(StepTrue, HalfPiYawVxAdvancesY) {
  const double dt = 0.1;
  const double vx = 0.5;
  Pose2D  s;  s.yaw = M_PI / 2.0;
  Twist2D u;  u.vx  = vx;

  Pose2D next = stepTrue(s, u, dt);

  // x_{k+1} = vx * cos(pi/2) * dt ~ 0
  // y_{k+1} = vx * sin(pi/2) * dt = vx*dt
  EXPECT_NEAR(next.x,   0.0,     kTol);
  EXPECT_NEAR(next.y,   vx * dt, kTol);
  EXPECT_NEAR(next.yaw, M_PI / 2.0, kTol);
}

// stepTrue and stepLinear DIFFER when s.yaw != theta0.
TEST(StepTrue, DiffersFromLinearWhenYawNeqTheta0) {
  const double dt = 0.1;
  // Robot faces pi/2 but we linearise about 0.
  Pose2D  s;  s.yaw = M_PI / 2.0;
  Twist2D u;  u.vx  = 1.0;

  Pose2D lin  = stepLinear(s, u, 0.0, dt);  // theta0=0
  Pose2D tru  = stepTrue(s, u, dt);         // uses actual yaw = pi/2

  // lin: x += vx*cos(0)*dt = dt, y += vx*sin(0)*dt = 0
  // true: x += vx*cos(pi/2)*dt ~ 0, y += vx*sin(pi/2)*dt = dt
  // They should differ.
  EXPECT_GT(std::abs(lin.x - tru.x) + std::abs(lin.y - tru.y), 1e-6);
}

// stepTrue and stepLinear ARE equal when s.yaw == theta0.
TEST(StepTrue, EqualToLinearWhenYawEqualsTheta0) {
  const double dt     = 0.1;
  const double theta0 = 1.2;
  Pose2D  s;  s.x = 0.5;  s.y = -0.3;  s.yaw = theta0;
  Twist2D u;  u.vx = 0.4;  u.vy = 0.1;  u.w = 0.2;

  Pose2D lin = stepLinear(s, u, theta0, dt);
  Pose2D tru = stepTrue(s, u, dt);

  EXPECT_NEAR(lin.x,   tru.x,   kTol);
  EXPECT_NEAR(lin.y,   tru.y,   kTol);
  EXPECT_NEAR(lin.yaw, tru.yaw, kTol);
}

// ---------------------------------------------------------------------------
// rolloutLinear: output size and integration correctness.

// Output size must equal us.size().
TEST(RolloutLinear, OutputSizeEqualsN) {
  const int N  = 5;
  const double theta0 = 0.0;
  const double dt     = 0.1;
  Pose2D s0;
  std::vector<Twist2D> us(static_cast<size_t>(N));  // all zero
  auto states = rolloutLinear(s0, us, theta0, dt);
  EXPECT_EQ(states.size(), static_cast<size_t>(N));
}

// Constant +vx for N steps from origin (theta0=0): x_N = N*vx*dt, y_N=0.
TEST(RolloutLinear, ConstantVxStraightLine) {
  const int    N      = 10;
  const double vx     = 0.5;
  const double dt     = 0.1;
  const double theta0 = 0.0;

  Pose2D s0;
  Twist2D u_cmd;  u_cmd.vx = vx;
  std::vector<Twist2D> us(static_cast<size_t>(N), u_cmd);

  auto states = rolloutLinear(s0, us, theta0, dt);

  EXPECT_EQ(states.size(), static_cast<size_t>(N));
  EXPECT_NEAR(states.back().x,   static_cast<double>(N) * vx * dt, kTol);
  EXPECT_NEAR(states.back().y,   0.0,                               kTol);
  EXPECT_NEAR(states.back().yaw, 0.0,                               kTol);
}

// Constant +w for N steps from origin (theta0=0): yaw_N = N*w*dt.
TEST(RolloutLinear, ConstantWAccumulatesYaw) {
  const int    N      = 20;
  const double w      = 0.8;
  const double dt     = 0.1;
  const double theta0 = 0.0;

  Pose2D s0;
  Twist2D u_cmd;  u_cmd.w = w;
  std::vector<Twist2D> us(static_cast<size_t>(N), u_cmd);

  auto states = rolloutLinear(s0, us, theta0, dt);

  EXPECT_NEAR(states.back().yaw,
              static_cast<double>(N) * w * dt,
              kTol);
}

// Empty us -> empty output.
TEST(RolloutLinear, EmptyInputReturnsEmpty) {
  Pose2D s0;
  std::vector<Twist2D> us;
  auto states = rolloutLinear(s0, us, 0.0, 0.1);
  EXPECT_TRUE(states.empty());
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
