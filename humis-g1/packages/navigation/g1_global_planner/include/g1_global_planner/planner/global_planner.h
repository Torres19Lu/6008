#pragma once

#include <vector>

#include "g1_global_planner/core/plan_types.h"
#include "g1_global_planner/core/planner_grid.h"

namespace g1_global_planner {

// High-level facade: world-frame Pose2D in, world-frame Pose2D path out.
// Handles start/goal snapping, A*, smoothing, and world-coordinate conversion.
class GlobalPlanner {
 public:
  explicit GlobalPlanner(const PlannerConfig& cfg);

  void               setConfig(const PlannerConfig& cfg);
  const PlannerConfig& config() const;

  // Plan a path from `start` to `goal` on the given grid.
  // - Converts start/goal world coords to map cells.
  // - If a cell is blocked and goal_snap_radius > 0, snaps to the nearest free
  //   cell within round(goal_snap_radius / resolution) cells via outward BFS.
  // - Runs A*, then smooths the cell path.
  // - Converts cells to world-frame Pose2D waypoints (cell centers). Each
  //   waypoint yaw = atan2 to the next; the last waypoint yaw = goal.yaw.
  // - Fills PlanResult.length_m with the sum of Euclidean segment lengths.
  PlanResult plan(const PlannerGrid& grid,
                  const Pose2D&      start,
                  const Pose2D&      goal) const;

  // Returns false if any consecutive pair of waypoints fails a lineOfSight check
  // on the current grid. Empty or single-point path returns true.
  // Used by the ROS node's replan trigger.
  bool pathStillValid(const PlannerGrid&        grid,
                      const std::vector<Pose2D>& path) const;

 private:
  // Return a COPY of `grid` with inscribed (blocked-but-not-true-lethal) cells
  // within goal_relax_radius of (gx, gy) downgraded to the highest traversable
  // value (lethal_threshold - 1). True-lethal (100) and unknown (-1) are untouched.
  // No-op when goal_relax_enable is false or the goal is off-grid.
  PlannerGrid relaxAroundGoal(const PlannerGrid& grid, double gx, double gy) const;

  PlannerConfig cfg_;
};

}  // namespace g1_global_planner
