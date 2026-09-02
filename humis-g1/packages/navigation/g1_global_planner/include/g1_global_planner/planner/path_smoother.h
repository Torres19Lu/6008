#pragma once

#include <vector>

#include "g1_global_planner/core/cost_model.h"
#include "g1_global_planner/core/plan_types.h"
#include "g1_global_planner/core/planner_grid.h"

namespace g1_global_planner {

// Returns true if the straight segment from cell a to cell b passes through
// no blocked cell (endpoints included). Uses a conservative supercover traversal
// so no blocked cell between a and b is skipped.
bool lineOfSight(const PlannerGrid& grid,
                 const CostModel&   cost,
                 Cell a,
                 Cell b);

// String-pull (greedy ray-cast) smoother.
// If !cfg.smooth_enable or the path has fewer than 3 cells, returns cells unchanged.
// Otherwise: walk from cells[0] as the anchor; advance to the farthest j that has
// an unobstructed line-of-sight from the anchor; push that cell; repeat until the
// goal (last cell) is reached. First and last cells are always preserved.
std::vector<Cell> smoothPath(const PlannerGrid& grid,
                             const CostModel&   cost,
                             const PlannerConfig& cfg,
                             const std::vector<Cell>& cells);

}  // namespace g1_global_planner
