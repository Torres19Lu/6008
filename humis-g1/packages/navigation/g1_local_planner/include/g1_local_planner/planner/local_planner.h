#pragma once

// local_planner.h -- ROS-free LocalPlanner facade for g1_local_planner.
// No ROS headers. Takes a CostmapView and std::vector<Pose2D> (the node
// converts the ROS messages), so the whole control pipeline stays unit-testable.
//
// Wires together the three core pieces for one control cycle:
//   1. path_reference::generateReference -- the arc-length rolling reference
//      with the goal-align heading switch + end clamp.
//   2. Mpc::solve -- the QP that produces u_0 + the predicted trajectory.
//   3. terminal / stop logic -- the GOAL_REACHED test and the zero-command
//      states for no goal / no costmap / no path / solver failure.
//
// Warm-started obstacle nominal: the facade KEEPS last cycle's predicted trajectory
// and feeds it back to Mpc::solve as the obstacle linearisation nominal on the next
// cycle (real-time-iteration SQP). Once the predicted path bows off an obstacle,
// the next cycle samples the costmap gradient where the path actually is, so
// avoidance is self-reinforcing across cycles. The nominal is only used when its
// size matches cfg.horizon (i.e. after a solved cycle); otherwise solve() falls
// back to sampling at the reference points.
//
// Slew warm start: the facade owns u_{-1} as last_cmd (the last applied u_0),
// reset to zero on every stop / non-tracking outcome so the slew constraint always
// starts from a real, in-box command after a stop.

#include <vector>

#include "g1_local_planner/core/costmap_view.h"
#include "g1_local_planner/core/footprint_safety.h"
#include "g1_local_planner/core/plan_types.h"
#include "g1_local_planner/core/robot_footprint.h"
#include "g1_local_planner/planner/mpc.h"

namespace g1_local_planner {

class LocalPlanner {
 public:
  explicit LocalPlanner(const MpcConfig& cfg);

  // Replace the live config. Also re-inits the embedded Mpc so a changed horizon /
  // weights / limits take effect on the next compute(). Clears the warm-start
  // nominal (its size may no longer match the new horizon) and resets last_cmd so
  // the slew constraint starts clean under the new limits.
  void setConfig(const MpcConfig& cfg);

  // Set the map-frame global path polyline (the node converts nav_msgs/Path).
  void setPath(const std::vector<Pose2D>& path);

  // Set the navigation goal pose (the node converts /move_base_simple/goal).
  void setGoal(const Pose2D& goal);

  // Set the local-window costmap snapshot (the node converts the OccupancyGrid).
  void setCostmap(const CostmapView& costmap);

  // Drop the goal (resets has_goal); compute() then returns NO_GOAL + zero.
  void clearGoal();

  // Run one control cycle. current = robot map pose (from TF in the node).
  // Returns the command in .applied, the predicted trajectory in .predicted, and
  // the controller state in .status. See the .cpp for the precedence of states.
  MpcResult compute(const Pose2D& current);

  // Const accessors (for the node's RViz/state publishing and for tests that
  // observe the warm-start nominal persisting across calls).
  const MpcConfig& config() const { return cfg_; }
  bool hasGoal() const { return has_goal_; }
  const Twist2D& lastCmd() const { return last_cmd_; }
  const std::vector<Pose2D>& lastPredicted() const { return last_predicted_; }

  // True when the footprint was built successfully from cfg_.footprint.
  // When false the safety filter is skipped (the node may log on mismatch).
  bool footprintOk() const { return footprint_ok_; }

 private:
  // Reset the open-loop warm-start state (last applied command + obstacle nominal)
  // on every stop / non-tracking outcome so the next solve starts from rest.
  void resetWarmStart();

  // (Re)build footprint_ from cfg_.footprint. Sets footprint_ok_ on success.
  // Called at the end of the ctor and at the end of setConfig().
  void rebuildFootprint();

  // Max normalized costmap cost over the footprint circle centres at pose `p`.
  // Used to gate near-obstacle strafe suppression (Track C / C3).
  double maxFootprintCost(const Pose2D& p, const CostmapView& cm) const;

  MpcConfig cfg_;
  Mpc mpc_;

  std::vector<Pose2D> path_;
  Pose2D goal_;
  bool has_goal_ = false;

  CostmapView costmap_;

  Twist2D last_cmd_;                    // u_{-1}, the last applied command (slew seed)
  std::vector<Pose2D> last_predicted_;  // warm-start obstacle nominal (real-time-iteration SQP)

  // Built from cfg_.footprint in the ctor and on setConfig().
  RobotFootprint footprint_;
  bool footprint_ok_ = false;

  // Stall-near-end tracking (Track A). stall_counter_ counts consecutive cycles
  // with no progress toward the path end while inside goal_align_radius;
  // stall_best_dist_ is the closest distance to the path end seen this approach
  // (a new best by reach_stall_progress resets the counter). Both reset on every
  // stop via resetWarmStart() and on a new goal.
  int    stall_counter_   = 0;
  double stall_best_dist_ = 1e9;
};

}  // namespace g1_local_planner
