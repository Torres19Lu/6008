// g1_local_planner/test/test_robot_footprint.cpp
#include <gtest/gtest.h>
#include <cmath>
#include "g1_local_planner/core/robot_footprint.h"

using g1_local_planner::FootprintConfig;
using g1_local_planner::RobotFootprint;

TEST(RobotFootprint, DerivesFiveCirclesAndRadii) {
  FootprintConfig cfg;  // defaults: r=0.15, N=5, W=0.5
  RobotFootprint fp; std::string err;
  ASSERT_TRUE(RobotFootprint::build(cfg, &fp, &err)) << err;
  ASSERT_EQ(fp.circles().size(), 5u);
  EXPECT_NEAR(fp.inscribedRadius(), 0.15, 1e-9);
  EXPECT_NEAR(fp.circumscribedRadius(), 0.25, 1e-9);  // 0.10 + 0.15
  // Outermost centers at +-(W/2 - r) = +-0.10, x=0.
  EXPECT_NEAR(fp.circles().front().y_body, -0.10, 1e-9);
  EXPECT_NEAR(fp.circles().back().y_body, 0.10, 1e-9);
  EXPECT_NEAR(fp.circles().front().x_body, 0.0, 1e-9);
}

TEST(RobotFootprint, RejectsBadConfig) {
  RobotFootprint fp; std::string err;
  FootprintConfig c0; c0.circle_count = 0;
  EXPECT_FALSE(RobotFootprint::build(c0, &fp, &err));
  FootprintConfig cr; cr.circle_radius = 0.0;
  EXPECT_FALSE(RobotFootprint::build(cr, &fp, &err));
  // circumscribed must not exceed lateral half-width by construction; a 1-circle
  // footprint is allowed (centered).
  FootprintConfig c1; c1.circle_count = 1;
  EXPECT_TRUE(RobotFootprint::build(c1, &fp, &err));
  EXPECT_NEAR(fp.circles().front().y_body, 0.0, 1e-9);
}

TEST(RobotFootprint, ExplicitCentersOverrideDerivation) {
  FootprintConfig cfg; cfg.circle_y = {-0.2, 0.0, 0.2};
  RobotFootprint fp; std::string err;
  ASSERT_TRUE(RobotFootprint::build(cfg, &fp, &err)) << err;
  ASSERT_EQ(fp.circles().size(), 3u);
  EXPECT_NEAR(fp.circumscribedRadius(), 0.35, 1e-9);  // 0.20 + 0.15
}

TEST(RobotFootprint, WorldSamplesIdentityYaw) {
  FootprintConfig cfg; cfg.circle_y = {-0.10, 0.10};
  RobotFootprint fp; std::string err;
  ASSERT_TRUE(RobotFootprint::build(cfg, &fp, &err)) << err;
  std::vector<std::pair<double,double>> out;
  fp.worldSamples(1.0, 2.0, 0.0, &out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0].first, 1.0, 1e-9);
  EXPECT_NEAR(out[0].second, 2.0 - 0.10, 1e-9);
  EXPECT_NEAR(out[1].second, 2.0 + 0.10, 1e-9);
}

TEST(RobotFootprint, WorldSamplesNinetyDegYawSwapsAxes) {
  FootprintConfig cfg; cfg.circle_y = {0.10};  // one circle at body +Y
  RobotFootprint fp; std::string err;
  ASSERT_TRUE(RobotFootprint::build(cfg, &fp, &err)) << err;
  std::vector<std::pair<double,double>> out;
  fp.worldSamples(0.0, 0.0, M_PI / 2.0, &out);
  ASSERT_EQ(out.size(), 1u);
  // body +Y maps to world -X under +90 deg yaw.
  EXPECT_NEAR(out[0].first, -0.10, 1e-9);
  EXPECT_NEAR(out[0].second, 0.0, 1e-9);
}

TEST(RobotFootprint, WorldSamplesAppends) {
  FootprintConfig cfg; cfg.circle_y = {0.0};
  RobotFootprint fp; std::string err;
  ASSERT_TRUE(RobotFootprint::build(cfg, &fp, &err)) << err;
  std::vector<std::pair<double,double>> out{{9.0, 9.0}};
  fp.worldSamples(0.0, 0.0, 0.0, &out);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0].first, 9.0, 1e-9);  // preserved
}

// A circle set that leaves a lateral gap (> 2*radius between adjacent centers)
// must be rejected so a zero-margin collision check is honest against the body.
TEST(RobotFootprint, RejectsLateralGap) {
  FootprintConfig cfg;
  cfg.circle_radius = 0.10;
  cfg.circle_y = {-0.30, 0.30};  // spacing 0.60 > 2*0.10 = 0.20 -> gap
  RobotFootprint fp;
  std::string err;
  EXPECT_FALSE(RobotFootprint::build(cfg, &fp, &err));
  EXPECT_FALSE(err.empty());
}

TEST(RobotFootprint, AcceptsDefaultGapFreeRow) {
  FootprintConfig cfg;  // 5 circles r=0.15, spacing 0.05 <= 0.30
  RobotFootprint fp;
  std::string err;
  EXPECT_TRUE(RobotFootprint::build(cfg, &fp, &err)) << err;
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
