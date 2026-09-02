// nav_node.cpp -- g1_nav_node: the thin ROS node that drives the g1_nav_core
// (NavFsm + ProgressWatchdog) as a navigation orchestrator.
// All decision logic lives in the core; the node does I/O, TF, the action server,
// the velocity mux, the service clients, and executes the FSM decision each tick.
//
// The node is the SINGLE writer of /cmd_vel: it publishes EVERY tick (zero when
// idle) so the locomotion watchdog always sees a fresh stream.
//
// Threading: AsyncSpinner(2), one mutex (mu_). The TF lookup runs OFF the lock
// (tf_buffer_ is thread-safe, last-good TF cache is timer-thread-only). Each tick:
//   (a) TF lookup off the lock,
//   (b) LOCK mu_: tick the core helpers, build Observations, run fsm_.step,
//       execute the core-side (ROS-free) intents, compute the mux Twist, then
//       snapshot every value the post-lock section needs,
//   (c) UNLOCK mu_: publish /cmd_vel + /nav/state, forward the goal, call the
//       enable/disable/arm/halt services, and call the action result /
//       feedback methods.
// A SimpleActionServer result method or a blocking service .call() is NEVER
// invoked while holding mu_ (the AS-lock/mu_ deadlock and blocking-under-lock are
// the two failure modes this discipline avoids).

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <g1_msgs/LocoStatus.h>
#include <g1_msgs/NavigateToAction.h>
#include <g1_nav/NavTuningConfig.h>

#include "g1_nav/core/fsm.h"
#include "g1_nav/core/nav_types.h"
#include "g1_nav/core/progress_watchdog.h"

namespace g1_nav {

class NavNode {
 public:
  NavNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : tf_listener_(tf_buffer_),
        fsm_(nav_cfg_),
        watchdog_(watchdog_cfg_),
        as_(nh, readActionName(pnh), false) {
    // ---- Interface names (params with code defaults). ----
    pnh.param<std::string>("goal_topic",        goal_topic_,        "/move_base_simple/goal");
    pnh.param<std::string>("local_state_topic", local_state_topic_, "/g1_local_planner/state");
    pnh.param<std::string>("track_cmd_topic",   track_cmd_topic_,   "/nav/cmd_vel_track");
    pnh.param<std::string>("loco_status_topic", loco_status_topic_, "/g1/loco_status");
    pnh.param<std::string>("cmd_vel_topic",     cmd_vel_topic_,     "/cmd_vel");
    pnh.param<std::string>("goal_out_topic",    goal_out_topic_,    "/nav/goal");
    pnh.param<std::string>("state_topic",       state_topic_,       "/nav/state");
    pnh.param<std::string>("enable_service",    enable_service_,    "/g1_local_planner/enable");
    pnh.param<std::string>("arm_service",       arm_service_,       "/g1/arm");
    pnh.param<std::string>("halt_service",      halt_service_,      "/g1/halt");

    // ---- Frames. ----
    pnh.param<std::string>("map_frame",        map_frame_,        "map");
    pnh.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");

    // ---- Rates / TF. ----
    pnh.param("rate",                rate_,                20.0);
    pnh.param("transform_tolerance", transform_tolerance_,  0.2);
    pnh.param("tf_timeout",          tf_timeout_,           1.0);
    pnh.param("feedback_rate",       feedback_rate_,        5.0);
    pnh.param("track_cmd_ttl",       track_cmd_ttl_,        0.5);
    pnh.param("local_state_ttl",     local_state_ttl_,      0.5);
    pnh.param("goal_ack_grace",      goal_ack_grace_,       0.15);
    pnh.param("service_connect_timeout", service_connect_timeout_, 0.1);

    // ---- Core configs (flat keys; each defaults to the struct code default). ----
    applyConfig(pnh);

    // ---- Publishers. ----
    cmd_pub_   = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    goal_pub_  = nh.advertise<geometry_msgs::PoseStamped>(goal_out_topic_, 1, /*latch=*/true);
    state_pub_ = nh.advertise<std_msgs::Int8>(state_topic_, 1, /*latch=*/true);

    // ---- Subscribers. ----
    goal_sub_        = nh.subscribe(goal_topic_,        1, &NavNode::goalCb,       this);
    local_state_sub_ = nh.subscribe(local_state_topic_, 1, &NavNode::localStateCb, this);
    track_cmd_sub_   = nh.subscribe(track_cmd_topic_,   1, &NavNode::trackCmdCb,    this);
    loco_status_sub_ = nh.subscribe(loco_status_topic_, 1, &NavNode::locoStatusCb,  this);

    // ---- Service clients (created once; non-persistent -- persistent silently
    //      breaks if the server restarts and is not auto-reconnected). ----
    enable_client_ = nh.serviceClient<std_srvs::SetBool>(enable_service_);
    arm_client_    = nh.serviceClient<std_srvs::Trigger>(arm_service_);
    halt_client_   = nh.serviceClient<std_srvs::Trigger>(halt_service_);

    // ---- Action server (callbacks run on spinner threads). ----
    as_.registerGoalCallback(boost::bind(&NavNode::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&NavNode::preemptCallback, this));
    as_.start();

    // ---- Dynamic reconfigure server (live-tunable knobs only). ----
    // The server fires one callback at construction time (the "initial" set). At
    // that point the structural startup params are already applied by applyConfig
    // above; reconfigureCb updates ONLY the live subset (FSM timeouts + auto_arm,
    // watchdog, recovery knobs) and does NOT clobber the structural params.
    reconfigure_server_.reset(
        new dynamic_reconfigure::Server<g1_nav::NavTuningConfig>(pnh));
    reconfigure_server_->setCallback(
        boost::bind(&NavNode::reconfigureCb, this, boost::placeholders::_1,
                    boost::placeholders::_2));

    // ---- Timer (the 20 Hz control loop). ----
    timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &NavNode::timerCb, this);

    ROS_INFO(
        "g1_nav up: action=%s goal=%s local_state=%s track_cmd=%s cmd_vel=%s "
        "state=%s goal_out=%s frame=%s base=%s rate=%.1f Hz auto_arm=%s",
        action_name_.c_str(), goal_topic_.c_str(), local_state_topic_.c_str(),
        track_cmd_topic_.c_str(), cmd_vel_topic_.c_str(), state_topic_.c_str(),
        goal_out_topic_.c_str(), map_frame_.c_str(), robot_base_frame_.c_str(),
        rate_, nav_cfg_.auto_arm ? "true" : "false");
  }

 private:
  // Read the action name before the member-init list constructs as_ (which needs it).
  // Caches into action_name_ so the rest of the node can log/use it.
  std::string readActionName(ros::NodeHandle& pnh) {
    pnh.param<std::string>("action_name", action_name_, "navigate_to");
    return action_name_;
  }

  // ------------------------------------------------------------------
  // Config: read all three core configs as flat keys (each defaulting to the
  // struct code default) and push them into the helpers under mu_. Live updates
  // to the tunable subset go through reconfigureCb (which calls setConfig on the
  // helpers directly); the structural params read here are set once at startup.
  // ------------------------------------------------------------------
  void applyConfig(ros::NodeHandle& pnh) {
    NavConfig nc;
    pnh.param("plan_timeout",   nc.plan_timeout,   nc.plan_timeout);
    pnh.param("replan_timeout", nc.replan_timeout, nc.replan_timeout);
    pnh.param("goal_timeout",   nc.goal_timeout,   nc.goal_timeout);
    pnh.param("no_path_grace",  nc.no_path_grace,  nc.no_path_grace);
    pnh.param("auto_arm",       nc.auto_arm,       nc.auto_arm);
    pnh.param("retry_budget",          nc.retry_budget,          nc.retry_budget);
    pnh.param("retry_replan_interval", nc.retry_replan_interval, nc.retry_replan_interval);

    WatchdogConfig wc;
    pnh.param("stuck_window", wc.stuck_window, wc.stuck_window);
    pnh.param("stuck_dist",   wc.stuck_dist,   wc.stuck_dist);

    std::lock_guard<std::mutex> lk(mu_);
    nav_cfg_      = nc;
    watchdog_cfg_ = wc;
    fsm_.setConfig(nav_cfg_);
    watchdog_.setConfig(watchdog_cfg_);
  }

  // ------------------------------------------------------------------
  // Dynamic reconfigure callback: update ONLY the live subset of the three
  // core configs. Structural params (topics, frames, rate, services) keep their
  // startup values. Called once at construction time (initial set) and then
  // on every rqt_reconfigure change.
  // ------------------------------------------------------------------
  void reconfigureCb(g1_nav::NavTuningConfig& cfg, uint32_t /*level*/) {
    std::lock_guard<std::mutex> lk(mu_);

    // Copy the live subset into the stored config structs, preserving structural
    // fields that are not exposed by NavTuningConfig.
    nav_cfg_.plan_timeout   = cfg.plan_timeout;
    nav_cfg_.replan_timeout = cfg.replan_timeout;
    nav_cfg_.goal_timeout   = cfg.goal_timeout;
    nav_cfg_.no_path_grace  = cfg.no_path_grace;
    nav_cfg_.auto_arm       = cfg.auto_arm;
    nav_cfg_.retry_budget          = cfg.retry_budget;
    nav_cfg_.retry_replan_interval = cfg.retry_replan_interval;

    watchdog_cfg_.stuck_window = cfg.stuck_window;
    watchdog_cfg_.stuck_dist   = cfg.stuck_dist;

    fsm_.setConfig(nav_cfg_);
    watchdog_.setConfig(watchdog_cfg_);
  }

  // ------------------------------------------------------------------
  // Subscriber callbacks (spinner threads; only touch shared state under mu_).
  // ------------------------------------------------------------------

  void localStateCb(const std_msgs::Int8::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mu_);
    local_state_       = static_cast<LocalState>(msg->data);
    have_local_state_  = true;
    local_state_stamp_ = ros::Time::now();  // freshness TTL (gated in the timer).
  }

  void trackCmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mu_);
    track_cmd_.vx       = msg->linear.x;
    track_cmd_.vy       = msg->linear.y;
    track_cmd_.w        = msg->angular.z;
    track_cmd_stamp_    = ros::Time::now();
    have_track_cmd_     = true;
  }

  void locoStatusCb(const g1_msgs::LocoStatus::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mu_);
    loco_halted_ = msg->halted;
    loco_mode_   = msg->mode;  // warn if commanding velocity while not in MODE_WALK.
  }

  // RViz "2D Nav Goal" convenience: ignored while an action goal is active.
  void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    Pose2D goal;
    if (!poseStampedToMap(*msg, goal)) {
      return;  // poseStampedToMap warns; drop a non-map / invalid RViz goal.
    }
    std::lock_guard<std::mutex> lk(mu_);
    if (action_goal_active_) {
      ROS_WARN_THROTTLE(2.0, "g1_nav: action goal active; ignoring RViz goal");
      return;
    }
    pending_new_goal_ = true;
    pending_goal_     = goal;
    // action_goal_active_ stays false: no AS result is ever called for an RViz goal.
  }

  // ------------------------------------------------------------------
  // Action lifecycle (SimpleActionServer; callbacks run on spinner threads).
  // ------------------------------------------------------------------

  void goalCallback() {
    auto g = as_.acceptNewGoal();  // auto-preempts any previous active goal.

    // Validate BEFORE setting new_goal so the FSM never sees a rejected goal.
    if (g->goal_type != g1_msgs::NavigateToGoal::GOAL_POSE) {
      g1_msgs::NavigateToResult res;
      res.success = false;
      res.outcome = g1_msgs::NavigateToResult::OUTCOME_REJECTED;
      res.distance_remaining = 0.0;
      res.message = "unsupported goal_type (only GOAL_POSE is implemented)";
      as_.setAborted(res, res.message);
      return;  // no active flag set.
    }

    Pose2D goal;
    if (!poseStampedToMap(g->target_pose, goal)) {
      g1_msgs::NavigateToResult res;
      res.success = false;
      res.outcome = g1_msgs::NavigateToResult::OUTCOME_REJECTED;
      res.distance_remaining = 0.0;
      res.message = "invalid pose (non-finite or untransformable goal frame)";
      as_.setAborted(res, res.message);
      return;  // no active flag set.
    }

    std::lock_guard<std::mutex> lk(mu_);
    pending_new_goal_   = true;
    pending_goal_       = goal;
    action_goal_active_ = true;
  }

  void preemptCallback() {
    // SimpleActionServer fires preempt both on an explicit client cancel AND when
    // a new goal arrives. A new goal already sets pending_new_goal_ (the FSM
    // supersedes), so only treat this as a cancel when no new goal is waiting.
    if (as_.isNewGoalAvailable()) {
      return;
    }
    std::lock_guard<std::mutex> lk(mu_);
    pending_cancel_ = true;
  }

  // ------------------------------------------------------------------
  // TF helper: look up map_frame_ <- source at stamp, falling back to the last
  // good transform on failure. Timer-thread-only state; no extra lock.
  // ------------------------------------------------------------------
  bool lookup(const std::string& source, const ros::Time& stamp,
              geometry_msgs::TransformStamped& out) {
    try {
      out                = tf_buffer_.lookupTransform(map_frame_, source, stamp,
                                                      ros::Duration(transform_tolerance_));
      last_tf_           = out;
      last_tf_valid_     = true;
      return true;
    } catch (const tf2::TransformException& e) {
      // Reuse the last good transform across a TRANSIENT dropout, but NEVER past
      // tf_timeout: an unbounded reuse would drive the robot on a frozen pose if
      // SLAM/TF dies. Past the cap, report "no pose" so the caller stops driving.
      if (last_tf_valid_ &&
          (ros::Time::now() - last_tf_.header.stamp).toSec() <= tf_timeout_) {
        ROS_WARN_THROTTLE(2.0, "g1_nav: TF %s<-%s failed (%s); reusing last good",
                          map_frame_.c_str(), source.c_str(), e.what());
        out = last_tf_;
        return true;
      }
      ROS_WARN_THROTTLE(2.0,
          "g1_nav: TF %s<-%s failed (%s); last good stale (>%.1fs) -> no pose",
          map_frame_.c_str(), source.c_str(), e.what(), tf_timeout_);
      return false;
    }
  }

  // Convert a PoseStamped into a map-frame Pose2D. Returns false (with a throttled
  // warn) on a non-finite pose or an untransformable goal frame. An empty frame_id
  // is assumed to be the map frame (mirrors the local planner convention).
  bool poseStampedToMap(const geometry_msgs::PoseStamped& in, Pose2D& out) {
    geometry_msgs::PoseStamped in_map;
    const std::string& frame_id = in.header.frame_id;
    if (frame_id.empty()) {
      ROS_WARN_THROTTLE(2.0, "g1_nav: goal has empty frame_id; assuming %s",
                        map_frame_.c_str());
      in_map = in;
    } else if (frame_id != map_frame_) {
      try {
        const geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
            map_frame_, frame_id, in.header.stamp, ros::Duration(transform_tolerance_));
        tf2::doTransform(in, in_map, tf);
      } catch (const tf2::TransformException& e) {
        ROS_WARN_THROTTLE(2.0, "g1_nav: goal TF %s<-%s failed (%s); dropping goal",
                          map_frame_.c_str(), frame_id.c_str(), e.what());
        return false;
      }
    } else {
      in_map = in;
    }

    const double x   = in_map.pose.position.x;
    const double y   = in_map.pose.position.y;
    const double yaw = tf2::getYaw(in_map.pose.orientation);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      ROS_WARN_THROTTLE(2.0, "g1_nav: goal pose is not finite; dropping goal");
      return false;
    }
    out.x   = x;
    out.y   = y;
    out.yaw = yaw;
    return true;
  }

  // Call a blocking service safely: skip (throttled warn) when the server is not
  // reachable so the timer never blocks on an absent service. MUST be called with
  // mu_ released.
  template <typename SrvT>
  void callServiceGuarded(ros::ServiceClient& client, const std::string& name,
                          SrvT& srv) {
    if (!client.exists() &&
        !client.waitForExistence(ros::Duration(service_connect_timeout_))) {
      ROS_WARN_THROTTLE(5.0, "g1_nav: service %s unavailable; skipping call",
                        name.c_str());
      return;
    }
    if (!client.call(srv)) {
      ROS_WARN_THROTTLE(5.0, "g1_nav: service %s call failed", name.c_str());
    }
  }

  // Build a map-frame PoseStamped from a Pose2D.
  geometry_msgs::PoseStamped poseStampedFrom(const Pose2D& p,
                                             const ros::Time& stamp) const {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = map_frame_;
    ps.header.stamp    = stamp;
    ps.pose.position.x = p.x;
    ps.pose.position.y = p.y;
    ps.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p.yaw);
    ps.pose.orientation = tf2::toMsg(q);
    return ps;
  }

  // ------------------------------------------------------------------
  // The 20 Hz timer: the control loop. Order is exact (see brief).
  // ------------------------------------------------------------------
  void timerCb(const ros::TimerEvent&) {
    const ros::Time now_ros = ros::Time::now();
    const double    now     = now_ros.toSec();

    // 1. TF lookup map->base_link OFF the lock (last-good reuse on miss).
    geometry_msgs::TransformStamped tf_base;
    bool   have_pose = false;
    Pose2D pose;
    if (lookup(robot_base_frame_, ros::Time(0), tf_base)) {
      have_pose = true;
      pose.x   = tf_base.transform.translation.x;
      pose.y   = tf_base.transform.translation.y;
      pose.yaw = tf2::getYaw(tf_base.transform.rotation);
    }

    // Post-lock snapshot variables (filled under the lock, used after release).
    geometry_msgs::Twist cmd_msg;
    NavState  out_state           = NavState::IDLE;
    bool      out_forward_goal    = false;
    bool      out_enable_local    = false;
    bool      out_disable_local   = false;
    bool      out_arm             = false;
    bool      out_halt            = false;
    bool      out_finish_action   = false;
    bool      out_action_active   = false;
    NavOutcome out_outcome        = NavOutcome::NONE;
    Pose2D    out_goal;
    Pose2D    out_pose            = pose;
    bool      out_have_pose       = have_pose;
    double    out_distance_remaining = 0.0;
    uint8_t   out_loco_mode       = g1_msgs::LocoStatus::MODE_UNKNOWN;

    // 2. LOCK mu_: tick the core, build Observations, step the FSM, execute the
    //    core-side intents, compute the mux Twist, then snapshot for post-lock.
    {
      std::lock_guard<std::mutex> lk(mu_);

      // a. Watchdog ticks only while tracking and only with a fresh pose.
      if (have_pose && fsm_.state() == NavState::TRACK) {
        watchdog_.update(pose, now);
      }

      // b. Build Observations, consuming (and clearing) the pending_* edges.
      Observations obs;
      obs.new_goal = pending_new_goal_;
      obs.goal     = pending_goal_;
      obs.cancel   = pending_cancel_;
      pending_new_goal_ = false;
      pending_cancel_   = false;

      // Gate local_state freshness before the FSM consumes it. On a goal-to-goal
      // preempt the local planner is NOT disabled, so until it processes the new goal
      // it keeps publishing the PREVIOUS goal's state; and if it stalls/dies its last
      // value latches forever. In both cases feed the FSM a neutral NO_PATH so a stale
      // TRACKING/GOAL_REACHED cannot drive a premature PLAN->TRACK or spurious SUCCEEDED.
      if (obs.new_goal) goal_handoff_until_ = now_ros + ros::Duration(goal_ack_grace_);
      const bool within_handoff = now_ros < goal_handoff_until_;
      const bool state_fresh =
          have_local_state_ && (now_ros - local_state_stamp_).toSec() <= local_state_ttl_;
      obs.local_state       = (within_handoff || !state_fresh) ? LocalState::NO_PATH
                                                               : local_state_;
      obs.costmap_available = have_local_state_ && (local_state_ != LocalState::NO_COSTMAP);
      obs.have_pose         = have_pose;
      obs.pose              = pose;
      obs.no_progress       = (fsm_.state() == NavState::TRACK) && watchdog_.noProgress(now);
      obs.loco_halted       = loco_halted_;

      // e. Run the FSM transition.
      const NavDecision d = fsm_.step(obs, now);

      // f. Execute the CORE-side intents (no ROS; safe under the lock).
      if (d.reset_watchdog) watchdog_.reset();

      // g. Compute the mux Twist.
      Twist2D mux_cmd;  // ZERO by default.
      switch (d.mux) {
        case MuxMode::RELAY_TRACK: {
          // Never relay a stale tracking command (safety): zero past the TTL.
          const bool fresh =
              have_track_cmd_ &&
              ((now_ros - track_cmd_stamp_).toSec() <= track_cmd_ttl_);
          if (fresh) mux_cmd = track_cmd_;
          break;
        }
        case MuxMode::ZERO:
        default:
          break;  // zero.
      }
      // Safety: never command motion without a fresh pose. If TF went stale
      // past tf_timeout (have_pose false), force zero regardless of mux mode -- as
      // the sole /cmd_vel writer, g1_nav must not drive open-loop on a frozen pose.
      if (!have_pose) mux_cmd = Twist2D{};
      cmd_msg.linear.x  = mux_cmd.vx;
      cmd_msg.linear.y  = mux_cmd.vy;
      cmd_msg.angular.z = mux_cmd.w;

      // h. Snapshot everything the post-lock section needs.
      out_state         = d.next_state;
      out_forward_goal  = d.forward_goal;
      out_enable_local  = d.enable_local;
      out_disable_local = d.disable_local;
      out_arm           = d.arm;
      out_halt          = d.halt;
      out_finish_action = d.finish_action;
      out_outcome       = d.outcome;
      out_action_active = action_goal_active_;
      out_goal          = fsm_active_goal_;       // last forwarded/active goal.
      // If a new goal was adopted this tick, that is the active goal to forward.
      if (obs.new_goal) {
        fsm_active_goal_ = obs.goal;
        out_goal         = obs.goal;
      }
      out_pose          = pose;
      out_have_pose     = have_pose;
      out_loco_mode     = loco_mode_;  // snapshot under the lock for the post-lock diag.

      // distance_remaining = straight-line (Euclidean) distance to the active goal
      // (not path length); 0 if no pose or no active goal.
      if (have_pose) {
        const double dx = fsm_active_goal_.x - pose.x;
        const double dy = fsm_active_goal_.y - pose.y;
        out_distance_remaining = std::sqrt(dx * dx + dy * dy);
      }

      // F1: clear action_goal_active_ here, inside the lock, on the same tick the
      // terminal result is dispatched. A new acceptNewGoal on a spinner thread that
      // runs between the result call and a post-unlock re-lock would have its
      // action_goal_active_=true wrongly clobbered by a deferred write; doing the
      // clear here prevents that race.
      if (d.finish_action && action_goal_active_) {
        action_goal_active_ = false;
      }
    }
    // 3. UNLOCK mu_. All ROS I/O happens here.

    // 3.1 Publish /cmd_vel ALWAYS (zero when not driving).
    cmd_pub_.publish(cmd_msg);

    // Surface "commanding velocity but not in WALK mode". If g1_nav is driving
    // (nonzero /cmd_vel) but locomotion is not MODE_WALK, the robot silently drops the
    // command (wrong FSM / not armed into walk). Name the real cause instead of letting
    // it look like a generic stuck that aborts ~30 s later with no diagnostic.
    if ((cmd_msg.linear.x != 0.0 || cmd_msg.linear.y != 0.0 || cmd_msg.angular.z != 0.0) &&
        out_loco_mode != g1_msgs::LocoStatus::MODE_WALK) {
      ROS_WARN_THROTTLE(2.0,
          "g1_nav: commanding velocity but locomotion mode=%u (not MODE_WALK=%u); "
          "/cmd_vel is being dropped -- the robot is not in a walk FSM",
          out_loco_mode, g1_msgs::LocoStatus::MODE_WALK);
    }

    // 3.2 Publish /nav/state every tick.
    std_msgs::Int8 state_msg;
    state_msg.data = static_cast<int8_t>(out_state);
    state_pub_.publish(state_msg);

    // 3.3 Forward the active goal to /nav/goal.
    if (out_forward_goal) {
      goal_pub_.publish(poseStampedFrom(out_goal, now_ros));
    }

    // 3.4 Enable / disable the local planner.
    if (out_enable_local) {
      std_srvs::SetBool srv;
      srv.request.data = true;
      callServiceGuarded(enable_client_, enable_service_, srv);
    }
    if (out_disable_local) {
      std_srvs::SetBool srv;
      srv.request.data = false;
      callServiceGuarded(enable_client_, enable_service_, srv);
    }

    // 3.5 Arm / halt (only set by the FSM when auto_arm).
    if (out_arm) {
      std_srvs::Trigger srv;
      callServiceGuarded(arm_client_, arm_service_, srv);
    }
    if (out_halt) {
      std_srvs::Trigger srv;
      callServiceGuarded(halt_client_, halt_service_, srv);
    }

    // 3.7 Action result: only when an action goal is active (RViz goals never
    //     call an AS result). The AS result method is called OUTSIDE mu_.
    //     action_goal_active_ is already cleared inside the lock above (F1).
    if (out_finish_action && out_action_active && as_.isActive()) {
      g1_msgs::NavigateToResult res;
      res.outcome = static_cast<uint8_t>(out_outcome);
      res.success = (out_outcome == NavOutcome::SUCCEEDED);
      res.final_pose = poseStampedFrom(out_have_pose ? out_pose : Pose2D{}, now_ros);
      res.distance_remaining = static_cast<float>(out_distance_remaining);
      res.message = outcomeMessage(out_outcome);
      if (out_outcome == NavOutcome::SUCCEEDED) {
        as_.setSucceeded(res, res.message);
      } else if (out_outcome == NavOutcome::PREEMPTED) {
        as_.setPreempted(res, res.message);
      } else {
        as_.setAborted(res, res.message);
      }
      // No post-unlock re-lock: action_goal_active_=false was set inside mu_ above.
    }

    // 3.8 Feedback at feedback_rate while an action goal is active. Skip on the
    //     terminal tick (out_finish_action) to avoid publishFeedback on a goal that
    //     just had its result dispatched (actionlib error / no-op on a terminal goal).
    if (out_action_active && !out_finish_action && as_.isActive() &&
        feedback_rate_ > 0.0 &&
        (now - last_feedback_time_) >= (1.0 / feedback_rate_)) {
      last_feedback_time_ = now;
      g1_msgs::NavigateToFeedback fb;
      fb.state            = static_cast<uint8_t>(out_state);
      fb.current_pose     = poseStampedFrom(out_have_pose ? out_pose : Pose2D{}, now_ros);
      fb.distance_remaining = static_cast<float>(out_distance_remaining);
      fb.recovery_attempt = 0;  // reserved: recovery mechanism removed
      fb.status           = stateName(out_state);
      as_.publishFeedback(fb);
    }
  }

  // ------------------------------------------------------------------
  // Small string helpers (telemetry only; no behavior).
  // ------------------------------------------------------------------
  static const char* stateName(NavState s) {
    switch (s) {
      case NavState::IDLE:      return "IDLE";
      case NavState::PLAN:      return "PLAN";
      case NavState::TRACK:     return "TRACK";
      case NavState::REPLAN:    return "REPLAN";
      case NavState::RECOVER:   return "RECOVER";  // reserved: never emitted
      case NavState::SUCCEEDED: return "SUCCEEDED";
      case NavState::ABORTED:   return "ABORTED";
      case NavState::RETRY: return "RETRY";
    }
    return "UNKNOWN";
  }

  static const char* outcomeMessage(NavOutcome o) {
    switch (o) {
      case NavOutcome::SUCCEEDED:                  return "goal reached";
      case NavOutcome::ABORTED_NO_PATH:            return "aborted: no path";
      case NavOutcome::ABORTED_STUCK:              return "aborted: stuck";
      case NavOutcome::ABORTED_TIMEOUT:            return "aborted: goal timeout";
      case NavOutcome::ABORTED_RECOVERY_EXHAUSTED: return "aborted: recovery exhausted";
      case NavOutcome::PREEMPTED:                  return "preempted";
      case NavOutcome::REJECTED:                   return "rejected";
      case NavOutcome::ABORTED_BLOCKED_TIMEOUT:    return "aborted: blocked (no path within retry budget)";
      case NavOutcome::NONE:                       return "";
    }
    return "";
  }

  // ------------------------------------------------------------------
  // ROS handles.
  // ------------------------------------------------------------------
  ros::Subscriber goal_sub_;
  ros::Subscriber local_state_sub_;
  ros::Subscriber track_cmd_sub_;
  ros::Subscriber loco_status_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  goal_pub_;
  ros::Publisher  state_pub_;
  ros::ServiceClient enable_client_;
  ros::ServiceClient arm_client_;
  ros::ServiceClient halt_client_;
  ros::Timer      timer_;
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<dynamic_reconfigure::Server<g1_nav::NavTuningConfig>>
      reconfigure_server_;

  // ------------------------------------------------------------------
  // Core configs + helpers. The configs are members so the member-init list can
  // hand them to the helper constructors before applyConfig overwrites them with
  // the param values. All three helpers are touched ONLY under mu_.
  // ------------------------------------------------------------------
  NavConfig      nav_cfg_;
  WatchdogConfig watchdog_cfg_;
  NavFsm            fsm_;
  ProgressWatchdog  watchdog_;

  // ------------------------------------------------------------------
  // Shared state (guarded by mu_ unless noted).
  // ------------------------------------------------------------------
  std::mutex mu_;
  LocalState local_state_      = LocalState::NO_GOAL;
  bool       have_local_state_ = false;
  ros::Time  local_state_stamp_;       // when local_state_ last arrived (freshness TTL).
  ros::Time  goal_handoff_until_;      // ignore local_state until this (new-goal ack grace).
  Twist2D    track_cmd_;
  ros::Time  track_cmd_stamp_;
  bool       have_track_cmd_   = false;
  bool       loco_halted_      = false;
  uint8_t    loco_mode_        = g1_msgs::LocoStatus::MODE_UNKNOWN;  // walk-mode gate.

  // Edge events from the callbacks (consumed and cleared in the timer).
  bool   pending_new_goal_ = false;
  Pose2D pending_goal_;
  bool   pending_cancel_   = false;

  // The active goal the FSM is pursuing (set when a new goal is adopted). Used to
  // forward to /nav/goal and to compute distance_remaining for feedback/result.
  Pose2D fsm_active_goal_;

  // action_name_ MUST be declared before as_: members are constructed in
  // declaration order, and as_'s initializer calls readActionName(pnh) which
  // writes into action_name_. Declaring it after as_ would write into an
  // unconstructed std::string (UB / SIGSEGV on startup when the param is set).
  std::string action_name_;

  // Action server. action_goal_active_ is true between acceptNewGoal and the
  // result call; it gates whether finish_action calls an AS result (RViz goals
  // run with it false).
  actionlib::SimpleActionServer<g1_msgs::NavigateToAction> as_;
  bool action_goal_active_ = false;

  // ------------------------------------------------------------------
  // Static params (written once in the constructor; read-only after).
  // ------------------------------------------------------------------
  std::string goal_topic_, local_state_topic_, track_cmd_topic_, loco_status_topic_;
  std::string cmd_vel_topic_, goal_out_topic_, state_topic_;
  std::string enable_service_, arm_service_, halt_service_;
  std::string map_frame_, robot_base_frame_;
  double rate_                    = 20.0;
  double transform_tolerance_     = 0.2;
  double tf_timeout_              = 1.0;  // max age of a reused last-good TF before "no pose"
  double feedback_rate_           = 5.0;
  double track_cmd_ttl_           = 0.5;
  double local_state_ttl_         = 0.5;   // max age of local_state before "not fresh"
  double goal_ack_grace_          = 0.15;  // ignore local_state this long after a new goal
  double service_connect_timeout_ = 0.1;

  // Feedback pacing + last-good TF (timer-thread-only; no extra lock).
  double last_feedback_time_ = 0.0;
  geometry_msgs::TransformStamped last_tf_;
  bool   last_tf_valid_ = false;
};

}  // namespace g1_nav

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_nav");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  g1_nav::NavNode node(nh, pnh);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
