#include <gtest/gtest.h>
#include <g1_msgs/SemanticObject.h>
#include <g1_msgs/SemanticObjectArray.h>
#include <g1_msgs/NavPoint.h>
#include <g1_msgs/LocoStatus.h>
#include <g1_msgs/SetMode.h>
#include <g1_msgs/NavigateToAction.h>
#include <g1_msgs/RelocQuality.h>
#include <g1_msgs/GatewayCapture.h>
#include <g1_msgs/Relocalize.h>

TEST(G1Msgs, ConstructSemanticObjectArray) {
  g1_msgs::SemanticObject obj;
  obj.id = 7u;
  obj.class_name = "chair";
  obj.confidence = 0.9f;
  obj.observation_count = 3u;
  obj.moved = false;

  g1_msgs::SemanticObjectArray arr;
  arr.header.frame_id = "map";
  arr.objects.push_back(obj);

  EXPECT_EQ(arr.objects.size(), 1u);
  EXPECT_EQ(arr.objects[0].class_name, "chair");
  EXPECT_EQ(arr.header.frame_id, "map");
}

TEST(G1Msgs, EnumConstantsExist) {
  EXPECT_EQ(g1_msgs::NavPoint::SOURCE_USER, 1u);
  EXPECT_EQ(g1_msgs::LocoStatus::MODE_WALK, 3u);
}

TEST(G1Msgs, ConstructSetModeService) {
  g1_msgs::SetMode::Request req;
  req.mode = "start";
  g1_msgs::SetMode::Response res;
  res.success = true;
  res.message = "ok";
  EXPECT_EQ(req.mode, "start");
  EXPECT_TRUE(res.success);
}

TEST(G1Msgs, NavigateToActionTypes) {
  g1_msgs::NavigateToGoal goal;
  g1_msgs::NavigateToResult result;
  g1_msgs::NavigateToFeedback feedback;

  // goal_type enum constants
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToGoal::GOAL_POSE),      0u);
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToGoal::GOAL_OBJECT_ID), 1u);
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToGoal::GOAL_NAV_POINT), 2u);

  // outcome enum constants
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToResult::OUTCOME_SUCCEEDED),               0u);
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToResult::OUTCOME_ABORTED_NO_PATH),         1u);
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToResult::OUTCOME_ABORTED_STUCK),           2u);
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToResult::OUTCOME_ABORTED_TIMEOUT),         3u);
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToResult::OUTCOME_ABORTED_RECOVERY_EXHAUSTED), 4u);
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToResult::OUTCOME_PREEMPTED),               5u);
  EXPECT_EQ(static_cast<uint8_t>(g1_msgs::NavigateToResult::OUTCOME_REJECTED),                6u);

  // default-constructed fields
  EXPECT_EQ(goal.goal_type, 0u);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.outcome, 0u);
  EXPECT_EQ(feedback.distance_remaining, 0.0f);
}

TEST(G1Msgs, GatewayCaptureTypes) {
  g1_msgs::RelocQuality q;
  q.inlier_ratio = 0.8; q.accepted = true;
  EXPECT_TRUE(q.accepted);

  g1_msgs::GatewayCapture::Request req;
  req.command = "begin"; req.to_map = "m2"; req.values = {0.1, -0.2};
  EXPECT_EQ(req.command, "begin");

  g1_msgs::Relocalize::Response res;
  res.quality.inlier_ratio = 0.5;          // new field exists
  EXPECT_DOUBLE_EQ(res.quality.inlier_ratio, 0.5);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
