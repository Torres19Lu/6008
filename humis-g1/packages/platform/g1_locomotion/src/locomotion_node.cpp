#include <array>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <g1_msgs/LocoStatus.h>
#include <g1_msgs/SetMode.h>

#include "g1_locomotion/loco_bridge.hpp"
#include "g1_locomotion/loco_types.hpp"

namespace {

bool loadJointMap(ros::NodeHandle& pnh,
                  std::vector<g1_locomotion::JointMapEntry>& out) {
  XmlRpc::XmlRpcValue list;
  if (!pnh.getParam("joint_map", list)) {
    ROS_FATAL("g1_locomotion: ~joint_map param missing");
    return false;
  }
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_FATAL("g1_locomotion: ~joint_map must be a list");
    return false;
  }
  for (int i = 0; i < list.size(); ++i) {
    XmlRpc::XmlRpcValue& e = list[i];
    if (!e.hasMember("name") || !e.hasMember("index")) {
      ROS_FATAL("g1_locomotion: joint_map[%d] needs 'name' and 'index'", i);
      return false;
    }
    g1_locomotion::JointMapEntry entry;
    entry.name = static_cast<std::string>(e["name"]);
    entry.sdk_index = static_cast<int>(e["index"]);
    out.push_back(entry);
  }
  return !out.empty();
}

}  // namespace

class LocomotionNode {
 public:
  LocomotionNode(ros::NodeHandle& nh, const g1_locomotion::LocoBridge::Config& cfg,
                 double control_rate, double joint_state_rate, double status_rate,
                 std::string joint_state_frame, std::string imu_frame)
      : bridge_(cfg),
        control_rate_(control_rate),
        joint_state_rate_(joint_state_rate),
        status_rate_(status_rate),
        joint_state_frame_(std::move(joint_state_frame)),
        imu_frame_(std::move(imu_frame)),
        nh_(nh) {}

  bool init() {
    if (!bridge_.init()) {
      ROS_FATAL("g1_locomotion: bridge init failed: %s",
                bridge_.lastError().c_str());
      return false;
    }
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("g1/body_imu", 10);
    status_pub_ = nh_.advertise<g1_msgs::LocoStatus>("g1/loco_status", 10);

    cmd_sub_ = nh_.subscribe("cmd_vel", 1, &LocomotionNode::onCmdVel, this);
    halt_srv_ =
        nh_.advertiseService("g1/halt", &LocomotionNode::onHalt, this);
    arm_srv_ = nh_.advertiseService("g1/arm", &LocomotionNode::onArm, this);
    mode_srv_ =
        nh_.advertiseService("g1/set_mode", &LocomotionNode::onSetMode, this);

    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                     &LocomotionNode::onControl, this);
    state_timer_ = nh_.createTimer(ros::Duration(1.0 / joint_state_rate_),
                                   &LocomotionNode::onState, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0 / status_rate_),
                                    &LocomotionNode::onStatus, this);
    return true;
  }

  // Soft-stop (zero velocity) on shutdown; the robot stays balanced in FSM 500.
  void shutdown() { bridge_.halt(); }

 private:
  void onCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    g1_locomotion::Velocity v;
    v.vx = msg->linear.x;
    v.vy = msg->linear.y;
    v.vyaw = msg->angular.z;
    bridge_.setVelocityTarget(v, ros::Time::now().toSec());
  }

  bool onHalt(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    const int ret = bridge_.halt();
    res.success = (ret == 0);
    res.message = (ret == 0)
                      ? "soft-stop: zero velocity, holding balanced (FSM 500)"
                      : ("halt SetVelocity error " + std::to_string(ret));
    ROS_WARN("g1_locomotion: HALT (soft-stop) -> zero velocity, ret=%d", ret);
    return true;
  }

  bool onArm(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    bridge_.arm();
    res.success = true;
    res.message =
        "armed: velocity streaming enabled (robot must be in a walk FSM to move)";
    ROS_WARN("g1_locomotion: ARM -> velocity streaming enabled");
    return true;
  }

  bool onSetMode(g1_msgs::SetMode::Request& req,
                 g1_msgs::SetMode::Response& res) {
    std::string msg;
    bool drops_robot = false;
    const int ret = bridge_.setMode(req.mode, msg, drops_robot);
    res.success = (ret == 0);
    res.message = msg;
    if (drops_robot) {
      ROS_ERROR("g1_locomotion: set_mode('%s') -> %s", req.mode.c_str(),
                msg.c_str());
    } else {
      ROS_WARN("g1_locomotion: set_mode('%s') -> %s", req.mode.c_str(),
               msg.c_str());
    }
    return true;
  }

  void onControl(const ros::TimerEvent& ev) {
    double dt = ev.last_real.isZero()
                    ? (1.0 / control_rate_)
                    : (ev.current_real - ev.last_real).toSec();
    if (!(dt > 0.0) || dt > 1.0) dt = 1.0 / control_rate_;  // NaN-safe
    bridge_.controlStep(ros::Time::now().toSec(), dt);
  }

  void onState(const ros::TimerEvent&) {
    g1_locomotion::JointStateData js;
    if (bridge_.latestJointState(js)) {
      sensor_msgs::JointState m;
      m.header.stamp = ros::Time::now();
      m.header.frame_id = joint_state_frame_;
      m.name = js.names;
      m.position = js.positions;
      joint_pub_.publish(m);
    }
    std::array<double, 4> q;
    std::array<double, 3> g;
    std::array<double, 3> a;
    if (bridge_.latestImu(q, g, a)) {
      sensor_msgs::Imu m;
      m.header.stamp = ros::Time::now();
      m.header.frame_id = imu_frame_;
      m.orientation.w = q[0];  // SDK quaternion order is [w, x, y, z]
      m.orientation.x = q[1];
      m.orientation.y = q[2];
      m.orientation.z = q[3];
      m.angular_velocity.x = g[0];
      m.angular_velocity.y = g[1];
      m.angular_velocity.z = g[2];
      m.linear_acceleration.x = a[0];
      m.linear_acceleration.y = a[1];
      m.linear_acceleration.z = a[2];
      imu_pub_.publish(m);
    }
  }

  void onStatus(const ros::TimerEvent&) {
    g1_msgs::LocoStatus m;
    m.header.stamp = ros::Time::now();
    // ROS bool is uint8_t; pollStatus takes bool& so use local bridge variables.
    bool halted = false;
    bridge_.pollStatus(m.mode, halted, m.error_code);
    m.halted = halted;
    m.battery_soc = 0.0f;  // not present in hg LowState_; best-effort
    status_pub_.publish(m);
  }

  g1_locomotion::LocoBridge bridge_;
  double control_rate_;
  double joint_state_rate_;
  double status_rate_;
  std::string joint_state_frame_;
  std::string imu_frame_;

  ros::NodeHandle nh_;
  ros::Publisher joint_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher status_pub_;
  ros::Subscriber cmd_sub_;
  ros::ServiceServer halt_srv_;
  ros::ServiceServer arm_srv_;
  ros::ServiceServer mode_srv_;
  ros::Timer control_timer_;
  ros::Timer state_timer_;
  ros::Timer status_timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_locomotion");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  g1_locomotion::LocoBridge::Config cfg;
  pnh.param<std::string>("network_interface", cfg.network_interface,
                         std::string());
  pnh.param("domain_id", cfg.domain_id, 0);
  pnh.param("client_timeout", cfg.client_timeout, 0.2);
  pnh.param("vx_max", cfg.vel_limits.vx_max, 0.5);
  pnh.param("vx_back_max", cfg.vel_limits.vx_back_max, 0.3);
  pnh.param("vy_max", cfg.vel_limits.vy_max, 0.3);
  pnh.param("vyaw_max", cfg.vel_limits.vyaw_max, 0.8);
  pnh.param("ax_max", cfg.accel_limits.ax_max, 0.5);
  pnh.param("ay_max", cfg.accel_limits.ay_max, 0.5);
  pnh.param("ayaw_max", cfg.accel_limits.ayaw_max, 1.0);
  pnh.param("command_ttl", cfg.command_ttl, 0.5);
  pnh.param("watchdog_timeout", cfg.watchdog_timeout, 0.3);
  pnh.param("deadzone_vx", cfg.deadzone.vx, 0.1);
  pnh.param("deadzone_vy", cfg.deadzone.vy, 0.1);
  pnh.param("deadzone_vyaw", cfg.deadzone.vyaw, 0.1);
  if (!pnh.getParam("allowed_fsm_ids", cfg.allowed_fsm_ids) ||
      cfg.allowed_fsm_ids.empty()) {
    cfg.allowed_fsm_ids = {500, 801};  // default walk FSMs (REGULAR_WALK/WALK_RUN)
  }

  double control_rate, joint_state_rate, status_rate;
  pnh.param("control_rate", control_rate, 10.0);
  pnh.param("joint_state_rate", joint_state_rate, 50.0);
  pnh.param("status_rate", status_rate, 2.0);

  std::string joint_state_frame, imu_frame;
  pnh.param<std::string>("joint_state_frame", joint_state_frame, std::string());
  pnh.param<std::string>("imu_frame", imu_frame, std::string("imu_in_pelvis"));

  if (!loadJointMap(pnh, cfg.joint_map)) return 1;
  if (cfg.network_interface.empty()) {
    ROS_FATAL(
        "g1_locomotion: network_interface empty (set $G1_NETWORK_INTERFACE)");
    return 1;
  }

  ROS_WARN("g1_locomotion: connecting to G1 on '%s' (domain %d)",
           cfg.network_interface.c_str(), cfg.domain_id);

  LocomotionNode node(nh, cfg, control_rate, joint_state_rate, status_rate,
                      joint_state_frame, imu_frame);
  if (!node.init()) return 1;

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  node.shutdown();
  return 0;
}
