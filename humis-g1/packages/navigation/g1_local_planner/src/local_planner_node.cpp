// ROS node for the linear MPC local trajectory tracker. I/O only: subscribes the
// global path (/nav/global_path), local costmap (/nav/local_costmap), and the
// navigation goal (/move_base_simple/goal), looks up the robot pose via TF
// (map<-base_link), drives the ROS-free LocalPlanner on a fixed-rate timer, and
// publishes /cmd_vel, /nav/local_plan, and ~state.
//
// Threading: AsyncSpinner(2), one mutex (mu_). The TF lookup runs OFF the lock
// (tf_buffer_ is thread-safe). All facade calls (compute, setConfig) and all shared
// state run UNDER mu_ so the reconfigure callback cannot race the timer. The message
// payload is assembled under the lock; publish happens AFTER releasing mu_.
// facade.compute() is STATEFUL (warm-start: last_predicted + last_cmd), so it MUST
// be serialised by mu_ -- never called concurrently.
//
// Enable gate: std_srvs/SetBool ~enable. When disabled, cmd_vel is published as
// ZERO (so a stale nonzero does not move the robot), but /nav/local_plan and ~state
// still publish so the operator can watch the predicted trajectory before arming.

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include "g1_local_planner/core/costmap_view.h"
#include "g1_local_planner/core/plan_types.h"
#include "g1_local_planner/planner/local_planner.h"
#include "g1_local_planner/LocalPlannerTuningConfig.h"

namespace g1_local_planner {
namespace {

// Raw costmap fields cached from nav_msgs/OccupancyGrid (ROS-free pod types so
// we can snapshot them under the lock without pulling in a ROS header from the core).
struct CostmapSnapshot {
  double resolution = 0.0;
  double origin_x   = 0.0;
  double origin_y   = 0.0;
  int    width      = 0;
  int    height     = 0;
  std::vector<int8_t> data;
  bool valid = false;
};

}  // namespace

class LocalPlannerNode {
 public:
  LocalPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : tf_listener_(tf_buffer_) {
    // Interface params.
    std::string path_topic, costmap_topic, goal_topic, cmd_topic,
                local_plan_topic;
    pnh.param<std::string>("path_topic",       path_topic,       "/nav/global_path");
    pnh.param<std::string>("costmap_topic",     costmap_topic,    "/nav/local_costmap");
    pnh.param<std::string>("goal_topic",        goal_topic,       "/move_base_simple/goal");
    pnh.param<std::string>("cmd_topic",         cmd_topic,        "/cmd_vel");
    pnh.param<std::string>("local_plan_topic",  local_plan_topic, "/nav/local_plan");
    pnh.param<std::string>("map_frame",         map_frame_,       "map");
    pnh.param<std::string>("robot_base_frame",  robot_base_frame_, "base_link");

    // Rate / TF params.
    pnh.param("update_rate",         update_rate_,         20.0);
    pnh.param("transform_tolerance", transform_tolerance_,  0.2);
    pnh.param("tf_timeout",          tf_timeout_,           1.0);

    // Enable gate.
    pnh.param("enable_on_start", enabled_, false);

    // MpcConfig fields: seeded from YAML so the planner is valid before the first
    // reconfigure callback fires (mirror the global planner param-loading pattern).
    pnh.param("horizon",               cfg_.horizon,               cfg_.horizon);
    pnh.param("dt",                    cfg_.dt,                    cfg_.dt);
    pnh.param("v_ref",                 cfg_.v_ref,                 cfg_.v_ref);
    pnh.param("max_vx",                cfg_.max_vx,                cfg_.max_vx);
    pnh.param("min_vx",                cfg_.min_vx,                cfg_.min_vx);
    pnh.param("max_vy",                cfg_.max_vy,                cfg_.max_vy);
    pnh.param("max_wz",                cfg_.max_wz,                cfg_.max_wz);
    pnh.param("acc_lim_x",            cfg_.acc_lim_x,            cfg_.acc_lim_x);
    pnh.param("acc_lim_y",            cfg_.acc_lim_y,            cfg_.acc_lim_y);
    pnh.param("acc_lim_theta",        cfg_.acc_lim_theta,        cfg_.acc_lim_theta);
    pnh.param("xy_goal_tolerance",    cfg_.xy_goal_tolerance,    cfg_.xy_goal_tolerance);
    pnh.param("yaw_goal_tolerance",   cfg_.yaw_goal_tolerance,   cfg_.yaw_goal_tolerance);
    pnh.param("goal_align_radius",    cfg_.goal_align_radius,    cfg_.goal_align_radius);
    pnh.param("reach_on_path_end",    cfg_.reach_on_path_end,    cfg_.reach_on_path_end);
    pnh.param("reach_stall_progress", cfg_.reach_stall_progress, cfg_.reach_stall_progress);
    pnh.param("reach_stall_cycles",   cfg_.reach_stall_cycles,   cfg_.reach_stall_cycles);
    pnh.param("reach_stall_radius",   cfg_.reach_stall_radius,   cfg_.reach_stall_radius);
    pnh.param("reach_goal_gap",       cfg_.reach_goal_gap,       cfg_.reach_goal_gap);
    pnh.param("goal_obstacle_relax_radius", cfg_.goal_obstacle_relax_radius, cfg_.goal_obstacle_relax_radius);
    pnh.param("goal_obstacle_relax_scale",  cfg_.goal_obstacle_relax_scale,  cfg_.goal_obstacle_relax_scale);
    pnh.param("a_lat_max",          cfg_.a_lat_max,          cfg_.a_lat_max);
    pnh.param("a_decel",            cfg_.a_decel,            cfg_.a_decel);
    pnh.param("curv_lookahead",     cfg_.curv_lookahead,     cfg_.curv_lookahead);
    pnh.param("heading_slow_start", cfg_.heading_slow_start, cfg_.heading_slow_start);
    pnh.param("heading_slow_full",  cfg_.heading_slow_full,  cfg_.heading_slow_full);
    pnh.param("heading_slow_floor", cfg_.heading_slow_floor, cfg_.heading_slow_floor);
    pnh.param("v_min_move",         cfg_.v_min_move,         cfg_.v_min_move);
    pnh.param("w_pos",                cfg_.w_pos,                cfg_.w_pos);
    pnh.param("w_yaw",                cfg_.w_yaw,                cfg_.w_yaw);
    pnh.param("w_obstacle",           cfg_.w_obstacle,           cfg_.w_obstacle);
    pnh.param("w_effort",             cfg_.w_effort,             cfg_.w_effort);
    pnh.param("w_lateral",            cfg_.w_lateral,            cfg_.w_lateral);
    pnh.param("w_rate",               cfg_.w_rate,               cfg_.w_rate);
    pnh.param("w_pos_terminal",       cfg_.w_pos_terminal,       cfg_.w_pos_terminal);
    pnh.param("w_yaw_terminal",       cfg_.w_yaw_terminal,       cfg_.w_yaw_terminal);
    pnh.param("obstacle_nominal_damping", cfg_.obstacle_nominal_damping, cfg_.obstacle_nominal_damping);
    pnh.param("vy_suppress_cost",     cfg_.vy_suppress_cost,     cfg_.vy_suppress_cost);
    pnh.param("vy_suppress_scale",    cfg_.vy_suppress_scale,    cfg_.vy_suppress_scale);
    pnh.param("lethal_cost",          cfg_.lethal_cost,          cfg_.lethal_cost);
    pnh.param("treat_unknown_as_obstacle",
              cfg_.treat_unknown_as_obstacle,
              cfg_.treat_unknown_as_obstacle);

    // Footprint: body-frame circle array. circle_y, if present, overrides the
    // derived even spacing.
    pnh.param("footprint_circle_radius", cfg_.footprint.circle_radius, cfg_.footprint.circle_radius);
    pnh.param("footprint_circle_count",  cfg_.footprint.circle_count,  cfg_.footprint.circle_count);
    pnh.param("footprint_lateral_width", cfg_.footprint.lateral_width, cfg_.footprint.lateral_width);
    pnh.param("footprint_forward_depth", cfg_.footprint.forward_depth, cfg_.footprint.forward_depth);
    pnh.getParam("footprint_circle_y",   cfg_.footprint.circle_y);  // optional explicit centers

    // Safety filter: post-MPC footprint guarantee.
    pnh.param("safety_filter_enabled",   cfg_.safety.enabled,       cfg_.safety.enabled);
    pnh.param("safety_lethal_value",     cfg_.safety.lethal_value,  cfg_.safety.lethal_value);
    pnh.param("safety_brake_margin",     cfg_.safety.brake_margin,  cfg_.safety.brake_margin);
    pnh.param("safety_footprint_collision_margin",
              cfg_.safety.footprint_collision_margin,
              cfg_.safety.footprint_collision_margin);
    pnh.param("safety_swept_subsamples", cfg_.safety.swept_subsamples, cfg_.safety.swept_subsamples);
    pnh.param("safety_treat_unknown_as_obstacle",
              cfg_.safety.treat_unknown_as_obstacle,
              cfg_.safety.treat_unknown_as_obstacle);
    pnh.param("control_dt",              cfg_.control_dt,           cfg_.control_dt);

    facade_ = std::make_unique<LocalPlanner>(cfg_);

    // The footprint is built inside the facade from cfg_.footprint; an invalid
    // config disables the safety filter silently in the core, so surface it here.
    if (!facade_->footprintOk()) {
      ROS_WARN("g1_local_planner: invalid robot footprint config; safety filter DISABLED. "
               "Check footprint_circle_radius (>0) and footprint_circle_count (>=1).");
    }

    // Publishers.
    cmd_pub_        = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1);
    local_plan_pub_ = nh.advertise<nav_msgs::Path>(local_plan_topic, 1);
    state_pub_      = pnh.advertise<std_msgs::Int8>("state", 1);

    // Subscribers.
    path_sub_    = nh.subscribe(path_topic,    1,
                                &LocalPlannerNode::pathCb, this);
    costmap_sub_ = nh.subscribe(costmap_topic, 1,
                                &LocalPlannerNode::costmapCb, this);
    goal_sub_    = nh.subscribe(goal_topic,    1,
                                &LocalPlannerNode::goalCb, this);

    // Enable gate service.
    enable_srv_ = pnh.advertiseService("enable",
                                       &LocalPlannerNode::enableCb, this);

    // Dynamic reconfigure. Fires once at construction, seeding the server defaults.
    // The YAML values have already been loaded above into cfg_.
    recfg_server_ = std::make_unique<
        dynamic_reconfigure::Server<LocalPlannerTuningConfig>>(pnh);
    recfg_server_->setCallback(
        boost::bind(&LocalPlannerNode::reconfigureCb, this, _1, _2));

    timer_ = nh.createTimer(ros::Duration(1.0 / update_rate_),
                            &LocalPlannerNode::updateTimerCb, this);

    ROS_INFO(
        "g1_local_planner up: path=%s costmap=%s goal=%s cmd=%s "
        "local_plan=%s frame=%s base=%s rate=%.1f Hz enabled=%s",
        path_topic.c_str(), costmap_topic.c_str(), goal_topic.c_str(),
        cmd_topic.c_str(), local_plan_topic.c_str(),
        map_frame_.c_str(), robot_base_frame_.c_str(),
        update_rate_, enabled_ ? "true" : "false");
  }

 private:
  // ------------------------------------------------------------------
  // Subscriber callbacks (run on spinner threads; only touch shared state
  // under the lock).
  // ------------------------------------------------------------------

  void pathCb(const nav_msgs::Path::ConstPtr& msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      ROS_WARN_THROTTLE(2.0,
          "g1_local_planner: path frame_id '%s' != map frame '%s'; "
          "assuming the global planner publishes map-frame paths",
          msg->header.frame_id.c_str(), map_frame_.c_str());
    }

    std::vector<Pose2D> path;
    path.reserve(msg->poses.size());
    for (const auto& ps : msg->poses) {
      Pose2D wp;
      wp.x   = ps.pose.position.x;
      wp.y   = ps.pose.position.y;
      wp.yaw = tf2::getYaw(ps.pose.orientation);
      path.push_back(wp);
    }

    std::lock_guard<std::mutex> lk(mu_);
    path_      = std::move(path);
    have_path_ = true;
    facade_->setPath(path_);
  }

  void costmapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      ROS_WARN_THROTTLE(2.0,
          "g1_local_planner: costmap frame_id '%s' != map frame '%s'",
          msg->header.frame_id.c_str(), map_frame_.c_str());
    }

    const size_t expected = size_t(msg->info.width) * size_t(msg->info.height);
    if (msg->data.size() != expected) {
      ROS_WARN_THROTTLE(5.0,
          "g1_local_planner: costmap data size %zu != width %u * height %u (%zu); "
          "discarding malformed grid",
          msg->data.size(), msg->info.width, msg->info.height, expected);
      return;
    }

    CostmapSnapshot snap;
    snap.resolution = msg->info.resolution;
    snap.origin_x   = msg->info.origin.position.x;
    snap.origin_y   = msg->info.origin.position.y;
    snap.width      = static_cast<int>(msg->info.width);
    snap.height     = static_cast<int>(msg->info.height);
    snap.data       = msg->data;
    snap.valid      = true;

    std::lock_guard<std::mutex> lk(mu_);
    costmap_snap_ = std::move(snap);
  }

  void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    Pose2D goal;
    geometry_msgs::PoseStamped goal_map;

    const std::string& frame_id = msg->header.frame_id;
    if (frame_id.empty()) {
      // Empty frame_id: assume map frame but warn so misconfigured senders are
      // visible in the log (mirrors the global planner pattern).
      ROS_WARN_THROTTLE(2.0,
          "g1_local_planner: goal has empty frame_id; assuming %s",
          map_frame_.c_str());
      goal_map = *msg;
    } else if (frame_id != map_frame_) {
      try {
        const geometry_msgs::TransformStamped tf =
            tf_buffer_.lookupTransform(map_frame_, frame_id,
                                       msg->header.stamp,
                                       ros::Duration(transform_tolerance_));
        tf2::doTransform(*msg, goal_map, tf);
      } catch (const tf2::TransformException& e) {
        ROS_WARN_THROTTLE(2.0,
            "g1_local_planner: goal TF %s<-%s failed (%s); dropping goal",
            map_frame_.c_str(), frame_id.c_str(), e.what());
        return;
      }
    } else {
      goal_map = *msg;
    }

    goal.x   = goal_map.pose.position.x;
    goal.y   = goal_map.pose.position.y;
    goal.yaw = tf2::getYaw(goal_map.pose.orientation);

    std::lock_guard<std::mutex> lk(mu_);
    facade_->setGoal(goal);
  }

  // ------------------------------------------------------------------
  // Enable gate service callback.
  // ------------------------------------------------------------------

  bool enableCb(std_srvs::SetBool::Request&  req,
                std_srvs::SetBool::Response& res) {
    std::lock_guard<std::mutex> lk(mu_);
    enabled_     = req.data;
    // A disabled controller has no active goal: drop it so the still-running
    // compute() reports NO_GOAL instead of a STALE GOAL_REACHED for the goal it
    // just finished. The coordinator (g1_nav) re-forwards the goal whenever it
    // re-enables us (its FSM pairs enable_local with forward_goal), so nothing
    // is lost. Without this, on the first tick after a NEW goal the coordinator
    // can read the previous goal's latched GOAL_REACHED and spuriously finish
    // (a far goal HALTs without moving).
    if (!req.data) facade_->clearGoal();
    res.success  = true;
    res.message  = req.data ? "enabled" : "disabled";
    ROS_INFO("g1_local_planner: %s", res.message.c_str());
    return true;
  }

  // ------------------------------------------------------------------
  // Reconfigure callback: update live knobs and push new config to the
  // facade UNDER the lock so it cannot race compute().
  // ------------------------------------------------------------------

  void reconfigureCb(LocalPlannerTuningConfig& c, uint32_t /*level*/) {
    std::lock_guard<std::mutex> lk(mu_);
    cfg_.v_ref                 = c.v_ref;
    cfg_.max_vx                = c.max_vx;
    cfg_.min_vx                = c.min_vx;
    cfg_.max_vy                = c.max_vy;
    cfg_.max_wz                = c.max_wz;
    cfg_.acc_lim_x             = c.acc_lim_x;
    cfg_.acc_lim_y             = c.acc_lim_y;
    cfg_.acc_lim_theta         = c.acc_lim_theta;
    cfg_.xy_goal_tolerance     = c.xy_goal_tolerance;
    cfg_.yaw_goal_tolerance    = c.yaw_goal_tolerance;
    cfg_.goal_align_radius     = c.goal_align_radius;
    cfg_.reach_on_path_end     = c.reach_on_path_end;
    cfg_.reach_stall_progress  = c.reach_stall_progress;
    cfg_.reach_stall_cycles    = c.reach_stall_cycles;
    cfg_.reach_stall_radius    = c.reach_stall_radius;
    cfg_.reach_goal_gap        = c.reach_goal_gap;
    cfg_.goal_obstacle_relax_radius = c.goal_obstacle_relax_radius;
    cfg_.goal_obstacle_relax_scale  = c.goal_obstacle_relax_scale;
    cfg_.a_lat_max             = c.a_lat_max;
    cfg_.a_decel               = c.a_decel;
    cfg_.curv_lookahead        = c.curv_lookahead;
    cfg_.heading_slow_start    = c.heading_slow_start;
    cfg_.heading_slow_full     = c.heading_slow_full;
    cfg_.heading_slow_floor    = c.heading_slow_floor;
    cfg_.v_min_move            = c.v_min_move;
    cfg_.w_pos                 = c.w_pos;
    cfg_.w_yaw                 = c.w_yaw;
    cfg_.w_obstacle            = c.w_obstacle;
    cfg_.w_effort              = c.w_effort;
    cfg_.w_lateral             = c.w_lateral;
    cfg_.w_rate                = c.w_rate;
    cfg_.w_pos_terminal        = c.w_pos_terminal;
    cfg_.w_yaw_terminal        = c.w_yaw_terminal;
    cfg_.obstacle_nominal_damping = c.obstacle_nominal_damping;
    cfg_.vy_suppress_cost      = c.vy_suppress_cost;
    cfg_.vy_suppress_scale     = c.vy_suppress_scale;
    cfg_.safety.enabled        = c.safety_filter_enabled;
    cfg_.safety.brake_margin   = c.safety_brake_margin;
    cfg_.safety.footprint_collision_margin = c.safety_footprint_collision_margin;
    facade_->setConfig(cfg_);
  }

  // ------------------------------------------------------------------
  // TF helper: look up map_frame_ <- source at stamp.
  // On failure falls back to last_out (if last_valid) with a throttled
  // warning. Returns false only when no cached transform exists yet.
  // (Timer-thread-only for the last_* state; no extra lock needed.)
  // ------------------------------------------------------------------

  bool lookup(const std::string& source, const ros::Time& stamp,
              geometry_msgs::TransformStamped& out,
              geometry_msgs::TransformStamped& last, bool& last_valid) {
    try {
      out        = tf_buffer_.lookupTransform(map_frame_, source, stamp,
                                              ros::Duration(transform_tolerance_));
      last       = out;
      last_valid = true;
      return true;
    } catch (const tf2::TransformException& e) {
      // Reuse the last good transform across a TRANSIENT dropout, but NEVER past
      // tf_timeout: an unbounded reuse would track / latch GOAL_REACHED on a frozen
      // pose if SLAM/TF dies. Past the cap, report failure so the node stops driving.
      if (last_valid &&
          (ros::Time::now() - last.header.stamp).toSec() <= tf_timeout_) {
        ROS_WARN_THROTTLE(2.0, "g1_local_planner: TF %s<-%s failed (%s); reusing last good",
                          map_frame_.c_str(), source.c_str(), e.what());
        out = last;
        return true;
      }
      ROS_WARN_THROTTLE(2.0,
          "g1_local_planner: TF %s<-%s failed (%s); last good stale (>%.1fs) -> no pose",
          map_frame_.c_str(), source.c_str(), e.what(), tf_timeout_);
      return false;
    }
  }

  // ------------------------------------------------------------------
  // Timer callback (20 Hz): the main control loop.
  //
  // Mutex discipline (mirrors g1_costmap + g1_global_planner pattern):
  //   (a) LOCK: snapshot have_path_, costmap_snap_ fields, enabled_
  //   (b) OFF LOCK: TF lookup map<-base_link (tf_buffer_ is thread-safe)
  //   (c) LOCK AGAIN: build CostmapView, push to facade.setCostmap,
  //                   call facade.compute(current_pose) -- MUST be under
  //                   the lock so reconfigureCb/setConfig cannot race it
  //   (d) release lock, THEN publish cmd_vel / local_plan / state
  //
  // facade.compute() and facade.setConfig() both mutate the facade's
  // warm-start state (last_predicted, last_cmd), so they MUST be
  // serialised by the SAME mutex mu_.
  // ------------------------------------------------------------------

  void updateTimerCb(const ros::TimerEvent&) {
    // (a) Snapshot under the lock.
    CostmapSnapshot cm_snap;
    bool enabled_snap;
    {
      std::lock_guard<std::mutex> lk(mu_);
      cm_snap      = costmap_snap_;
      enabled_snap = enabled_;
    }

    // (b) TF lookup OFF the lock (tf_buffer_ is thread-safe; last-good TF
    //     cache is timer-thread-only so no extra lock needed).
    geometry_msgs::TransformStamped tf_now;
    const bool have_pose = lookup(robot_base_frame_, ros::Time(0), tf_now,
                                  last_tf_base_, last_tf_base_valid_);

    MpcResult result;  // default zero command; status set below.
    if (!have_pose) {
      // No fresh pose (TF never acquired, or stale past tf_timeout): do NOT run the
      // MPC on an absent/frozen pose -- it could track blind or latch a false
      // GOAL_REACHED. Report STUCK (zero cmd) so the coordinator (g1_nav) reacts
      // instead of reading a silently-stale state.
      ROS_WARN_THROTTLE(2.0,
          "g1_local_planner: no fresh map<-%s TF; emitting STUCK (zero cmd)",
          robot_base_frame_.c_str());
      result.status  = ControllerState::STUCK;
      result.applied = Twist2D{};
      result.solved  = false;
    } else {
      Pose2D current;
      current.x   = tf_now.transform.translation.x;
      current.y   = tf_now.transform.translation.y;
      current.yaw = tf2::getYaw(tf_now.transform.rotation);

      // (c) LOCK AGAIN: feed costmap snapshot into the facade and run compute().
      std::lock_guard<std::mutex> lk(mu_);

      // Build CostmapView from the snapshot (this is cheap, no ROS dependency).
      // Belt-and-suspenders: re-verify the size matches before constructing the
      // view, since CostmapView's constructor throws std::invalid_argument on a
      // mismatch.  The callback already rejects malformed grids so this path
      // should never be false, but it guards against future code paths that might
      // bypass the ingestion check.
      if (cm_snap.valid &&
          cm_snap.data.size() == size_t(cm_snap.width) * size_t(cm_snap.height)) {
        CostmapView view(cm_snap.resolution,
                         cm_snap.origin_x,
                         cm_snap.origin_y,
                         cm_snap.width,
                         cm_snap.height,
                         cm_snap.data,
                         cfg_.lethal_cost,
                         cfg_.treat_unknown_as_obstacle);
        facade_->setCostmap(view);
      }

      // compute() serialised under mu_ (facade is stateful: warm-start nominal
      // and last_cmd are mutated here; setConfig also mutates them).
      result = facade_->compute(current);
    }
    // (d) Release lock, then publish.

    // cmd_vel: zero when disabled OR when not TRACKING.
    geometry_msgs::Twist cmd_msg;
    if (enabled_snap && result.status == ControllerState::TRACKING) {
      cmd_msg.linear.x  = result.applied.vx;
      cmd_msg.linear.y  = result.applied.vy;
      cmd_msg.angular.z = result.applied.w;
    }
    // linear.z / angular.x / angular.y remain 0 (default-constructed).
    cmd_pub_.publish(cmd_msg);

    // /nav/local_plan: predicted MPC trajectory (published even when disabled).
    nav_msgs::Path plan_msg;
    plan_msg.header.frame_id = map_frame_;
    plan_msg.header.stamp    = ros::Time::now();
    plan_msg.poses.reserve(result.predicted.size());
    for (const Pose2D& wp : result.predicted) {
      geometry_msgs::PoseStamped ps;
      ps.header = plan_msg.header;
      ps.pose.position.x = wp.x;
      ps.pose.position.y = wp.y;
      ps.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, wp.yaw);
      ps.pose.orientation = tf2::toMsg(q);
      plan_msg.poses.push_back(ps);
    }
    local_plan_pub_.publish(plan_msg);

    // ~state: ControllerState as Int8.
    std_msgs::Int8 state_msg;
    state_msg.data = static_cast<int8_t>(result.status);
    state_pub_.publish(state_msg);
  }

  // ------------------------------------------------------------------
  // ROS handles.
  // ------------------------------------------------------------------
  ros::Subscriber path_sub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  local_plan_pub_;
  ros::Publisher  state_pub_;
  ros::ServiceServer enable_srv_;
  ros::Timer      timer_;
  tf2_ros::Buffer             tf_buffer_;
  tf2_ros::TransformListener  tf_listener_;
  std::unique_ptr<dynamic_reconfigure::Server<LocalPlannerTuningConfig>> recfg_server_;

  // ------------------------------------------------------------------
  // Shared state (guarded by mu_).
  // ------------------------------------------------------------------
  std::mutex mu_;
  std::vector<Pose2D>  path_;
  bool have_path_      = false;
  CostmapSnapshot      costmap_snap_;
  bool enabled_        = false;

  // ------------------------------------------------------------------
  // Facade and its live config (both guarded by mu_; compute + setConfig
  // MUST run under the lock).
  // ------------------------------------------------------------------
  std::unique_ptr<LocalPlanner> facade_;
  MpcConfig cfg_;

  // ------------------------------------------------------------------
  // Static params (written once in the constructor; read-only after).
  // ------------------------------------------------------------------
  std::string map_frame_;
  std::string robot_base_frame_;
  double update_rate_         = 20.0;
  double transform_tolerance_ = 0.2;
  double tf_timeout_ = 1.0;  // max age of a reused last-good TF before "no pose"

  // Last-good TF for the robot base frame (timer-thread only; no extra lock).
  geometry_msgs::TransformStamped last_tf_base_;
  bool last_tf_base_valid_ = false;
};

}  // namespace g1_local_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_local_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  g1_local_planner::LocalPlannerNode node(nh, pnh);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
