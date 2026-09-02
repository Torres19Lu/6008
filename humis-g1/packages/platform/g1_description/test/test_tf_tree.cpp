#include <cmath>
#include <string>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

namespace {

// Block until a transform is available or the timeout elapses.
bool waitForTransform(const tf2_ros::Buffer& buffer, const std::string& target,
                      const std::string& source, double timeout_s) {
  const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_s);
  while (ros::ok() && ros::Time::now() < deadline) {
    if (buffer.canTransform(target, source, ros::Time(0))) return true;
    ros::Duration(0.05).sleep();
  }
  return buffer.canTransform(target, source, ros::Time(0));
}

}  // namespace

class TfTreeTest : public ::testing::Test {
 protected:
  TfTreeTest() : listener_(buffer_) {}
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

// The full chain from base_link down to the sensors and the camera bridge must
// resolve, plus the dynamically published base_footprint.
TEST_F(TfTreeTest, TreeIsConnected) {
  ASSERT_TRUE(waitForTransform(buffer_, "base_link", "torso_link", 20.0));
  EXPECT_TRUE(buffer_.canTransform("base_link", "pelvis", ros::Time(0)));
  EXPECT_TRUE(buffer_.canTransform("base_link", "mid360_link", ros::Time(0)));
  EXPECT_TRUE(buffer_.canTransform("base_link", "d435_link", ros::Time(0)));
  EXPECT_TRUE(buffer_.canTransform("d435_link", "camera_link", ros::Time(0)));
  // base_footprint is published by base_footprint_publisher (driven by
  // odom->base_link), not robot_state_publisher, so it can lag the static URDF
  // frames; wait for it rather than sampling immediately.
  EXPECT_TRUE(waitForTransform(buffer_, "base_link", "base_footprint", 20.0));
}

// base_link -> pelvis is identity (REP-105 kinematic root, zero offset to pelvis).
TEST_F(TfTreeTest, BaseLinkToPelvisIsIdentity) {
  ASSERT_TRUE(waitForTransform(buffer_, "base_link", "pelvis", 20.0));
  auto tf = buffer_.lookupTransform("base_link", "pelvis", ros::Time(0));
  EXPECT_NEAR(tf.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.translation.y, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.translation.z, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.rotation.w, 1.0, 1e-6);
}

// The fixed torso_link -> mid360_link extrinsic matches Appendix A regardless of
// waist motion (it is below the moving waist joint, so check from torso_link).
TEST_F(TfTreeTest, Mid360ExtrinsicMatchesUrdf) {
  ASSERT_TRUE(waitForTransform(buffer_, "torso_link", "mid360_link", 20.0));
  auto tf = buffer_.lookupTransform("torso_link", "mid360_link", ros::Time(0));
  EXPECT_NEAR(tf.transform.translation.x, 0.0002835, 1e-6);
  EXPECT_NEAR(tf.transform.translation.y, 0.00003, 1e-6);
  EXPECT_NEAR(tf.transform.translation.z, 0.41618, 1e-6);
}

// The camera bridge defaults to identity (the extrinsic is overridden once the camera is calibrated).
TEST_F(TfTreeTest, CameraBridgeIsIdentity) {
  ASSERT_TRUE(waitForTransform(buffer_, "d435_link", "camera_link", 20.0));
  auto tf = buffer_.lookupTransform("d435_link", "camera_link", ros::Time(0));
  EXPECT_NEAR(tf.transform.translation.x, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.translation.z, 0.0, 1e-6);
  EXPECT_NEAR(tf.transform.rotation.w, 1.0, 1e-6);
}

// The animated waist_yaw must actually move torso_link relative to base_link:
// sample the yaw over time and require meaningful variation.
TEST_F(TfTreeTest, WaistYawAnimatesTorso) {
  ASSERT_TRUE(waitForTransform(buffer_, "base_link", "torso_link", 20.0));
  double min_yaw = 1e9, max_yaw = -1e9;
  const ros::Time deadline = ros::Time::now() + ros::Duration(3.0);
  while (ros::ok() && ros::Time::now() < deadline) {
    auto tf = buffer_.lookupTransform("base_link", "torso_link", ros::Time(0));
    const double yaw = tf2::getYaw(tf.transform.rotation);
    min_yaw = std::min(min_yaw, yaw);
    max_yaw = std::max(max_yaw, yaw);
    ros::Duration(0.05).sleep();
  }
  EXPECT_GT(max_yaw - min_yaw, 0.2);  // radians of observed sweep
}

// base_footprint must land on the ground (z = 0 in odom) given the fake
// odom->base_link at z = 0.793.
TEST_F(TfTreeTest, FootprintProjectsToGround) {
  ASSERT_TRUE(waitForTransform(buffer_, "odom", "base_footprint", 20.0));
  auto tf = buffer_.lookupTransform("odom", "base_footprint", ros::Time(0));
  EXPECT_NEAR(tf.transform.translation.z, 0.0, 1e-3);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_tf_tree");
  ros::NodeHandle nh;  // keep the node alive for the listener
  ros::AsyncSpinner spinner(1);
  spinner.start();
  const int ret = RUN_ALL_TESTS();
  spinner.stop();
  return ret;
}
