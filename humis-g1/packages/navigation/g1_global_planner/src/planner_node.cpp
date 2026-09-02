// ROS node for the 2D A* global path planner. I/O only: subscribes the fused
// costmap (/nav/costmap) and the RViz goal (/move_base_simple/goal), looks up
// the robot pose via TF (map<-base_link), drives the ROS-free GlobalPlanner on
// a fixed-rate timer, and publishes the result as /nav/global_path. The TF
// lookup runs off the lock (tf_buffer_ is thread-safe); all planner calls and
// planner-related flags run under mu_ to exclude concurrent reconfigureCb.

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include "g1_global_planner/core/plan_types.h"
#include "g1_global_planner/core/planner_grid.h"
#include "g1_global_planner/planner/global_planner.h"
#include "g1_global_planner/PlannerTuningConfig.h"

namespace g1_global_planner {
namespace {

const char* planStatusName(PlanStatus s) {
  switch (s) {
    case PlanStatus::SUCCESS:   return "SUCCESS";
    case PlanStatus::NO_PATH:   return "NO_PATH";
    case PlanStatus::BAD_START: return "BAD_START";
    case PlanStatus::BAD_GOAL:  return "BAD_GOAL";
    case PlanStatus::CAPPED:    return "CAPPED";
  }
  return "UNKNOWN";
}

}  // namespace

class GlobalPlannerNode {
 public:
  GlobalPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : tf_listener_(tf_buffer_) {
    // Interface params.
    std::string costmap_topic, goal_topic, path_topic;
    pnh.param<std::string>("costmap_topic", costmap_topic, "/nav/costmap");
    pnh.param<std::string>("goal_topic",    goal_topic,    "/move_base_simple/goal");
    pnh.param<std::string>("path_topic",    path_topic,    "/nav/global_path");
    pnh.param<std::string>("map_frame",        map_frame_,        "map");
    pnh.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");

    // Rate / TF params.
    pnh.param("update_rate",         update_rate_,         5.0);
    pnh.param("transform_tolerance", transform_tolerance_, 0.2);
    pnh.param("tf_timeout",          tf_timeout_,          1.0);
    pnh.param("replan_fail_tolerance", replan_fail_tolerance_, 2);

    // PlannerConfig fields: seeded from YAML so the planner is valid before the
    // first reconfigure callback fires.
    pnh.param("lethal_threshold",      cfg_.lethal_threshold,      cfg_.lethal_threshold);
    pnh.param("allow_unknown",         cfg_.allow_unknown,         cfg_.allow_unknown);
    pnh.param("inflation_cost_weight", cfg_.inflation_cost_weight, cfg_.inflation_cost_weight);
    pnh.param("heuristic_weight",      cfg_.heuristic_weight,      cfg_.heuristic_weight);
    pnh.param("smooth_enable",         cfg_.smooth_enable,         cfg_.smooth_enable);
    pnh.param("goal_snap_radius",      cfg_.goal_snap_radius,      cfg_.goal_snap_radius);
    pnh.param("goal_relax_enable",     cfg_.goal_relax_enable,     cfg_.goal_relax_enable);
    pnh.param("goal_relax_radius",     cfg_.goal_relax_radius,     cfg_.goal_relax_radius);
    pnh.param("max_expansions",        cfg_.max_expansions,        cfg_.max_expansions);
    pnh.param("allow_corner_cutting",  cfg_.allow_corner_cutting,  cfg_.allow_corner_cutting);

    // Node-level replan trigger.
    pnh.param("replan_on_block", replan_on_block_, true);

    planner_ = std::make_unique<GlobalPlanner>(cfg_);

    // Publisher (latched so a new subscriber immediately gets the last path).
    path_pub_ = nh.advertise<nav_msgs::Path>(path_topic, 1, /*latch=*/true);

    // Subscribers.
    costmap_sub_ = nh.subscribe(costmap_topic, 1,
                                &GlobalPlannerNode::costmapCb, this);
    goal_sub_    = nh.subscribe(goal_topic, 1,
                                &GlobalPlannerNode::goalCb, this);

    // Dynamic reconfigure (fires once immediately at construction, seeding cfg_
    // from the server defaults - the YAML values above have already set cfg_).
    recfg_server_ = std::make_unique<
        dynamic_reconfigure::Server<PlannerTuningConfig>>(pnh);
    recfg_server_->setCallback(
        boost::bind(&GlobalPlannerNode::reconfigureCb, this, _1, _2));

    timer_ = nh.createTimer(ros::Duration(1.0 / update_rate_),
                            &GlobalPlannerNode::updateTimerCb, this);

    ROS_INFO(
        "g1_global_planner up: costmap=%s goal=%s path=%s frame=%s "
        "base=%s rate=%.1f Hz lethal=%d",
        costmap_topic.c_str(), goal_topic.c_str(), path_topic.c_str(),
        map_frame_.c_str(), robot_base_frame_.c_str(),
        update_rate_, cfg_.lethal_threshold);
  }

 private:
  // ------------------------------------------------------------------
  // Subscriber callbacks (run on spinner threads; only touch shared state
  // under the lock).
  // ------------------------------------------------------------------

  void costmapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    auto grid = std::make_shared<PlannerGrid>(
        msg->info.resolution,
        msg->info.origin.position.x,
        msg->info.origin.position.y,
        static_cast<int>(msg->info.width),
        static_cast<int>(msg->info.height),
        msg->data);
    std::lock_guard<std::mutex> lk(mu_);
    grid_     = std::move(grid);
    new_grid_ = true;
  }

  void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    Pose2D goal;
    geometry_msgs::PoseStamped goal_map;

    // Transform goal into map frame if necessary.
    const std::string& frame_id = msg->header.frame_id;
    if (frame_id.empty()) {
      // Empty frame_id: assume map frame but warn so misconfigured senders are
      // visible in the log.
      ROS_WARN_THROTTLE(2.0,
          "g1_global_planner: goal has empty frame_id; assuming %s",
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
            "goal TF %s<-%s failed (%s); dropping goal",
            map_frame_.c_str(), frame_id.c_str(), e.what());
        return;
      }
    } else {
      goal_map = *msg;
    }

    goal.x   = goal_map.pose.position.x;
    goal.y   = goal_map.pose.position.y;
    goal.yaw = tf2::getYaw(goal_map.pose.orientation);

    {
      std::lock_guard<std::mutex> lk(mu_);
      goal_      = goal;
      have_goal_ = true;
      new_goal_  = true;
    }
    // Force an immediate (event-driven) replan instead of waiting up to the
    // failsafe period for the next tick. setPeriod reschedules THIS timer; a single
    // timer's callbacks are serialized, so there is no concurrency with a periodic
    // fire and the last-good TF cache stays timer-thread-only. The tick restores the
    // failsafe cadence at its top.
    timer_.setPeriod(ros::Duration(0.0), /*reset=*/true);
  }

  // ------------------------------------------------------------------
  // Reconfigure callback.
  // ------------------------------------------------------------------

  void reconfigureCb(PlannerTuningConfig& c, uint32_t /*level*/) {
    std::lock_guard<std::mutex> lk(mu_);
    cfg_.lethal_threshold      = c.lethal_threshold;
    cfg_.allow_unknown         = c.allow_unknown;
    cfg_.inflation_cost_weight = c.inflation_cost_weight;
    cfg_.heuristic_weight      = c.heuristic_weight;
    cfg_.smooth_enable         = c.smooth_enable;
    cfg_.goal_snap_radius      = c.goal_snap_radius;
    cfg_.goal_relax_enable     = c.goal_relax_enable;
    cfg_.goal_relax_radius     = c.goal_relax_radius;
    replan_on_block_           = c.replan_on_block;
    planner_->setConfig(cfg_);
  }

  // ------------------------------------------------------------------
  // TF helper: look up map_frame_ <- source at stamp.
  // On failure, falls back to last_out (if last_valid) with a throttled
  // warning. Returns false only when no cached transform exists.
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
      // tf_timeout: an unbounded reuse would plan from a frozen start pose if
      // SLAM/TF dies. Past the cap, report failure so the tick skips planning.
      if (last_valid &&
          (ros::Time::now() - last.header.stamp).toSec() <= tf_timeout_) {
        ROS_WARN_THROTTLE(2.0, "TF %s<-%s failed (%s); reusing last good",
                          map_frame_.c_str(), source.c_str(), e.what());
        out = last;
        return true;
      }
      ROS_WARN_THROTTLE(2.0,
          "TF %s<-%s failed (%s); last good stale (>%.1fs) -> no start pose",
          map_frame_.c_str(), source.c_str(), e.what(), tf_timeout_);
      return false;
    }
  }

  // ------------------------------------------------------------------
  // Timer callback: TF lookup runs off the lock (tf_buffer_ is thread-safe);
  // all planner calls and planner-related flags run under mu_ (matching the
  // costmap_node pattern: heavy work under the lock, publish after release).
  // ------------------------------------------------------------------

  void updateTimerCb(const ros::TimerEvent&) {
    // Restore the failsafe cadence: goalCb may have shortened the period to force an
    // immediate (event-driven) replan; reset it here so we keep a steady failsafe and
    // never busy-loop (reset=false: set the period for the next fire, keep schedule).
    timer_.setPeriod(ros::Duration(1.0 / update_rate_), /*reset=*/false);

    // Snapshot grid + goal availability. Do NOT clear the new_* edge flags yet: if we
    // bail before planning (no grid/goal/pose) they must survive so the next tick
    // retries -- otherwise a goal arriving during a TF dropout (stale -> no pose)
    // would be lost.
    std::shared_ptr<PlannerGrid> grid;
    bool have_goal;
    {
      std::lock_guard<std::mutex> lk(mu_);
      grid      = grid_;
      have_goal = have_goal_;
    }

    if (!grid || !have_goal) return;

    // Look up current robot pose off the lock (tf_buffer_ is thread-safe; the
    // last-good TF cache is timer-thread-only so it needs no extra lock).
    geometry_msgs::TransformStamped tf_now;
    if (!lookup(robot_base_frame_, ros::Time(0), tf_now,
                last_tf_base_, last_tf_base_valid_)) {
      ROS_WARN_THROTTLE(2.0,
          "g1_global_planner: no map<-%s transform yet; skipping tick",
          robot_base_frame_.c_str());
      return;
    }

    Pose2D start;
    start.x   = tf_now.transform.translation.x;
    start.y   = tf_now.transform.translation.y;
    start.yaw = tf2::getYaw(tf_now.transform.rotation);

    // All planner calls and planner-related shared flags run under mu_ so that
    // reconfigureCb (which calls planner_->setConfig) cannot run concurrently
    // with plan() or pathStillValid() (both read the planner's internal cfg_).
    // The message is built under the lock; publish happens after release.
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = map_frame_;
    path_msg.header.stamp    = ros::Time::now();

    bool publish_path = false;
    bool success      = false;
    std::size_t wp_count    = 0;
    double      length_m    = 0.0;
    int         expansions  = 0;
    PlanStatus  plan_status = PlanStatus::NO_PATH;

    {
      std::lock_guard<std::mutex> lk(mu_);

      // Re-read goal + grid + edge flags under the lock (a newer goal/grid may have
      // arrived since the first snapshot) and CLEAR the edge flags now that we have a
      // pose and will evaluate/plan this tick.
      grid                = grid_;
      const Pose2D goal   = goal_;
      const bool   new_goal = new_goal_;
      const bool   new_grid = new_grid_;
      new_goal_ = false;
      new_grid_ = false;

      // Evaluate should_plan: have_valid_path_/last_path_ are shared state and
      // pathStillValid() reads the planner's internal cfg_.
      const bool should_plan =
          new_goal ||
          (new_grid && !have_valid_path_) ||
          (replan_on_block_ && have_valid_path_ &&
           !planner_->pathStillValid(*grid, last_path_));

      if (!should_plan) return;

      PlanResult result = planner_->plan(*grid, start, goal);
      plan_status       = result.status;

      if (result.status == PlanStatus::SUCCESS) {
        path_msg.poses.reserve(result.path.size());
        for (const Pose2D& wp : result.path) {
          geometry_msgs::PoseStamped ps;
          ps.header = path_msg.header;
          ps.pose.position.x  = wp.x;
          ps.pose.position.y  = wp.y;
          ps.pose.position.z  = 0.0;
          tf2::Quaternion q;
          q.setRPY(0.0, 0.0, wp.yaw);
          ps.pose.orientation = tf2::toMsg(q);
          path_msg.poses.push_back(ps);
        }
        last_path_          = result.path;
        have_valid_path_    = true;
        replan_fail_streak_ = 0;
        success             = true;
        wp_count            = result.path.size();
        length_m            = result.length_m;
        expansions          = result.expansions;
        publish_path        = true;
      } else if (!new_goal && have_valid_path_ &&
                 ++replan_fail_streak_ <= replan_fail_tolerance_) {
        // A TRANSIENT failure on an established goal. Keep the last good latched
        // path (the local planner is the fast reactor for transient obstacles) rather
        // than wiping it to empty after a single failed tick. publish_path stays false.
      } else {
        // New-goal failure, or a persistent failure past the tolerance: signal NO_PATH
        // by publishing the empty path and dropping the cached path.
        have_valid_path_    = false;
        replan_fail_streak_ = 0;
        publish_path        = true;
      }
    }  // release mu_ before publish

    if (publish_path) {
      path_pub_.publish(path_msg);
      if (success) {
        ROS_INFO_THROTTLE(5.0,
            "g1_global_planner: path found, waypoints=%zu length=%.2f m expansions=%d",
            wp_count, length_m, expansions);
      } else {
        ROS_WARN_THROTTLE(2.0, "g1_global_planner: planning failed: %s",
                          planStatusName(plan_status));
      }
    } else {
      ROS_WARN_THROTTLE(2.0,
          "g1_global_planner: transient plan failure (%s); keeping last good path "
          "(streak %d/%d)",
          planStatusName(plan_status), replan_fail_streak_, replan_fail_tolerance_);
    }
  }

  // ------------------------------------------------------------------
  // ROS handles.
  // ------------------------------------------------------------------
  ros::Subscriber costmap_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher  path_pub_;
  ros::Timer      timer_;
  tf2_ros::Buffer             tf_buffer_;
  tf2_ros::TransformListener  tf_listener_;
  std::unique_ptr<dynamic_reconfigure::Server<PlannerTuningConfig>> recfg_server_;

  // ------------------------------------------------------------------
  // Shared state (guarded by mu_).
  // ------------------------------------------------------------------
  std::mutex mu_;
  std::shared_ptr<PlannerGrid> grid_;
  Pose2D goal_;
  bool have_goal_      = false;
  bool new_goal_       = false;
  bool new_grid_       = false;
  bool replan_on_block_ = true;

  // ------------------------------------------------------------------
  // Planner and its live config.
  // ------------------------------------------------------------------
  std::unique_ptr<GlobalPlanner> planner_;
  PlannerConfig cfg_;

  // ------------------------------------------------------------------
  // Path cache (only written/read in the timer callback and under mu_
  // for the have_valid_path_ flag).
  // ------------------------------------------------------------------
  std::vector<Pose2D> last_path_;
  bool have_valid_path_ = false;

  // ------------------------------------------------------------------
  // Static params (written once in the constructor; read-only after).
  // ------------------------------------------------------------------
  std::string map_frame_;
  std::string robot_base_frame_;
  double update_rate_         = 5.0;
  double transform_tolerance_ = 0.2;
  double tf_timeout_ = 1.0;  // max age of a reused last-good TF before "no start pose"
  int    replan_fail_tolerance_ = 2;  // consecutive same-goal plan failures tolerated
  int    replan_fail_streak_    = 0;  // current consecutive-failure count (keeps last path)

  // Last-good TF for the robot base frame (timer-thread only; no extra lock).
  geometry_msgs::TransformStamped last_tf_base_;
  bool last_tf_base_valid_ = false;
};

}  // namespace g1_global_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_global_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  g1_global_planner::GlobalPlannerNode node(nh, pnh);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
