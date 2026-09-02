#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>

#include "g1_description/base_footprint.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_footprint_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string odom_frame, base_frame, footprint_frame;
  double publish_rate;
  pnh.param<std::string>("odom_frame", odom_frame, "odom");
  pnh.param<std::string>("base_frame", base_frame, "base_link");
  pnh.param<std::string>("footprint_frame", footprint_frame, "base_footprint");
  pnh.param<double>("publish_rate", publish_rate, 50.0);

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener(buffer);
  tf2_ros::TransformBroadcaster broadcaster;

  ros::Rate rate(publish_rate);
  while (ros::ok()) {
    ros::spinOnce();
    geometry_msgs::TransformStamped odom_T_base_msg;
    try {
      odom_T_base_msg = buffer.lookupTransform(odom_frame, base_frame, ros::Time(0));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "base_footprint: waiting for %s->%s (%s)",
                        odom_frame.c_str(), base_frame.c_str(), ex.what());
      rate.sleep();
      continue;
    }

    const Eigen::Isometry3d odom_T_base = tf2::transformToEigen(odom_T_base_msg);
    const Eigen::Isometry3d base_T_footprint =
        g1_description::computeBaseToFootprint(odom_T_base);

    geometry_msgs::TransformStamped out = tf2::eigenToTransform(base_T_footprint);
    // Stamp with now(), not the source stamp. base_footprint is rebroadcast at
    // publish_rate, which can exceed the odom->base_link update rate (a SLAM odom at
    // the LiDAR rate) or face a constant-stamp source (a static_transform_publisher
    // odom returns the same stamp every poll). Reusing the source stamp then emits
    // duplicate-stamped /tf and trips tf2 TF_REPEATED_DATA. now() keeps every
    // broadcast uniquely stamped and the frame fresh for fast, slow, and static
    // sources alike; base_footprint is a derived ground-projection frame, so the
    // small stamp-vs-source skew is immaterial to its consumers.
    out.header.stamp = ros::Time::now();
    out.header.frame_id = base_frame;
    out.child_frame_id = footprint_frame;
    broadcaster.sendTransform(out);

    rate.sleep();
  }
  return 0;
}
