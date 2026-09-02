#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace g1_global_planner {

// Integer grid cell coordinate.
struct Cell {
  int x = 0;
  int y = 0;
  bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
  bool operator!=(const Cell& o) const { return !(*this == o); }
};

// Metric pose in the map frame.
struct Pose2D {
  double x   = 0.0;
  double y   = 0.0;
  double yaw = 0.0;
};

// Outcome of a planning call.
enum class PlanStatus {
  SUCCESS,   // path found and returned
  NO_PATH,   // open set exhausted - goal unreachable
  BAD_START, // start cell out-of-bounds or blocked and not snappable
  BAD_GOAL,  // goal cell out-of-bounds or blocked and not snappable
  CAPPED,    // expansion limit hit before goal was reached
};

// Knobs for the core planner (no ROS / frame / topic params here).
struct PlannerConfig {
  int    lethal_threshold      = 99;    // block inscribed(99) AND lethal(100) for travel
  bool   allow_unknown         = false; // false -> unknown (-1) is blocked
  double inflation_cost_weight = 3.0;   // step-cost penalty scale for inflation cells (1..98)
  double heuristic_weight      = 1.0;   // 1.0 = optimal A*; >1 = weighted / greedy
  bool   smooth_enable         = true;
  double goal_snap_radius      = 0.3;   // m; snap blocked start/goal to nearest free cell (0 = off)
  int    max_expansions        = 0;     // 0 = unlimited; else cap node expansions
  bool   allow_corner_cutting  = false; // false -> diagonal needs both shared orthogonals free
  // Goal-local relax (Track B): within goal_relax_radius of the goal, treat
  // inscribed(99) as traversable-but-costly (downgraded to lethal_threshold-1) so the
  // path can approach a goal near an obstacle. Only true-lethal(100) stays blocked
  // there. The local footprint filter (g1_local_planner) is the safety guarantee.
  bool   goal_relax_enable     = true;
  double goal_relax_radius     = 0.6;   // m; relax disk around the goal (0 = off)
};

// Return value of a full planning call.
struct PlanResult {
  PlanStatus         status;
  std::vector<Pose2D> path;
  int                expansions = 0;
  double             length_m   = 0.0;
};

}  // namespace g1_global_planner
