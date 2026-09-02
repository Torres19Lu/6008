#pragma once

#include <vector>

#include "g1_global_planner/core/cost_model.h"
#include "g1_global_planner/core/plan_types.h"
#include "g1_global_planner/core/planner_grid.h"

namespace g1_global_planner {

// Raw cell-space A* result (world conversion happens in GlobalPlanner).
struct AStarResult {
  PlanStatus        status;
  std::vector<Cell> path;       // start..goal inclusive on SUCCESS
  int               expansions = 0;
};

// 8-connected A* on a PlannerGrid.
// Edge cost = move_dist * cost.stepCost(neighbor_cell_value).
// Heuristic = cfg.heuristic_weight * octile(dx, dy).
// Diagonal moves are blocked when !cfg.allow_corner_cutting and either shared
// orthogonal neighbor is blocked.
// Returns BAD_START / BAD_GOAL / CAPPED / NO_PATH / SUCCESS.
AStarResult astar(const PlannerGrid& grid,
                  const CostModel&   cost,
                  const PlannerConfig& cfg,
                  Cell start,
                  Cell goal);

}  // namespace g1_global_planner
