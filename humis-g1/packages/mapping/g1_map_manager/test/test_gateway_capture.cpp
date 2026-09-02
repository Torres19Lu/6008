#include <gtest/gtest.h>

#include <string>

#include <Eigen/Geometry>

#include "g1_map_manager/core/gateway_capture.h"

using namespace g1_map_manager;

namespace {
Eigen::Isometry3d xPose(double x) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation().x() = x;
  return p;
}
RelocQual good() { RelocQual q; q.inlier_ratio = 0.9; q.accepted = true; return q; }
}  // namespace

TEST(CaptureMachine, HappyPathBeginLoadCommit) {
  CaptureMachine cm;
  std::string msg;
  EXPECT_EQ(cm.state(), CaptureState::IDLE);
  ASSERT_TRUE(cm.begin("m1", "m2", "stairs", xPose(3.0), good(), msg)) << msg;
  EXPECT_EQ(cm.state(), CaptureState::BEGUN);
  EXPECT_EQ(cm.fromMap(), "m1");
  EXPECT_EQ(cm.toMap(), "m2");
  ASSERT_TRUE(cm.load(msg)) << msg;
  EXPECT_EQ(cm.state(), CaptureState::LOADED);

  GatewayEdge edge;
  ASSERT_TRUE(cm.commit(xPose(7.0), good(), edge, msg)) << msg;
  EXPECT_EQ(cm.state(), CaptureState::IDLE);            // hands off to the editor
  EXPECT_EQ(edge.from, "m1");
  EXPECT_EQ(edge.to, "m2");
  EXPECT_EQ(edge.label, "stairs");
  EXPECT_EQ(edge.source, "captured");
  EXPECT_DOUBLE_EQ(edge.pose_in_from.translation().x(), 3.0);
  EXPECT_DOUBLE_EQ(edge.pose_in_to.translation().x(), 7.0);
}

TEST(CaptureMachine, OutOfOrderRejected) {
  CaptureMachine cm;
  std::string msg;
  GatewayEdge edge;
  EXPECT_FALSE(cm.load(msg));                           // no begin
  EXPECT_FALSE(cm.commit(xPose(1.0), good(), edge, msg));
  ASSERT_TRUE(cm.begin("m1", "m2", "", xPose(0.0), good(), msg));
  EXPECT_FALSE(cm.begin("m1", "m3", "", xPose(0.0), good(), msg));  // already pending
  EXPECT_FALSE(cm.commit(xPose(1.0), good(), edge, msg));           // not loaded yet
}

TEST(CaptureMachine, RejectedCommitKeepsLoadedForRetry) {
  CaptureMachine cm;
  std::string msg;
  GatewayEdge edge;
  ASSERT_TRUE(cm.begin("m1", "m2", "", xPose(0.0), good(), msg));
  ASSERT_TRUE(cm.load(msg));
  RelocQual bad; bad.inlier_ratio = 0.1; bad.accepted = false;
  EXPECT_FALSE(cm.commit(xPose(7.0), bad, edge, msg));
  EXPECT_EQ(cm.state(), CaptureState::LOADED);          // retry without reload
  ASSERT_TRUE(cm.commit(xPose(7.0), good(), edge, msg));
  EXPECT_EQ(cm.state(), CaptureState::IDLE);
}

TEST(CaptureMachine, DiscardFromAnyState) {
  CaptureMachine cm;
  std::string msg;
  ASSERT_TRUE(cm.begin("m1", "m2", "", xPose(0.0), good(), msg));
  ASSERT_TRUE(cm.discard(msg));
  EXPECT_EQ(cm.state(), CaptureState::IDLE);
  ASSERT_TRUE(cm.begin("m1", "m2", "", xPose(0.0), good(), msg));
  ASSERT_TRUE(cm.load(msg));
  ASSERT_TRUE(cm.discard(msg));
  EXPECT_EQ(cm.state(), CaptureState::IDLE);
  EXPECT_FALSE(cm.discard(msg));                         // nothing to discard
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
