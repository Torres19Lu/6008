#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "fake_joint_states");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  urdf::Model model;
  if (!model.initParam("robot_description")) {
    ROS_FATAL("fake_joint_states: could not parse robot_description");
    return 1;
  }

  std::vector<std::string> joints;
  for (const auto& kv : model.joints_) {
    const auto& joint = kv.second;
    if (joint->type == urdf::Joint::REVOLUTE ||
        joint->type == urdf::Joint::CONTINUOUS ||
        joint->type == urdf::Joint::PRISMATIC) {
      joints.push_back(kv.first);
    }
  }
  ROS_INFO("fake_joint_states: publishing %zu movable joints", joints.size());

  std::string animated_joint;
  double amplitude, frequency_hz, rate_hz;
  pnh.param<std::string>("animated_joint", animated_joint, "waist_yaw_joint");
  pnh.param<double>("amplitude", amplitude, 1.0);
  pnh.param<double>("frequency_hz", frequency_hz, 0.2);
  pnh.param<double>("rate", rate_hz, 50.0);

  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  ros::Rate rate(rate_hz);
  const ros::Time start = ros::Time::now();
  while (ros::ok()) {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    const double t = (msg.header.stamp - start).toSec();
    const double animated_pos = amplitude * std::sin(2.0 * M_PI * frequency_hz * t);
    for (const auto& name : joints) {
      msg.name.push_back(name);
      msg.position.push_back(name == animated_joint ? animated_pos : 0.0);
    }
    pub.publish(msg);
    rate.sleep();
  }
  return 0;
}
