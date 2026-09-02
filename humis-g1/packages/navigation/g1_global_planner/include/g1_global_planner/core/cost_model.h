#pragma once

#include <cstdint>

#include "g1_global_planner/core/plan_types.h"

namespace g1_global_planner {

// Translates raw OccupancyGrid cell values to planner cost semantics.
// Constructed from a PlannerConfig; the config is copied at construction time.
class CostModel {
 public:
  explicit CostModel(const PlannerConfig& cfg) : cfg_(cfg) {}

  // Returns true if a cell is impassable (planner must not enter it).
  // Lethal: v >= lethal_threshold.
  // Unknown: v < 0 and !allow_unknown.
  bool blocked(int8_t v) const {
    if (v < 0) return !cfg_.allow_unknown;
    return v >= static_cast<int8_t>(cfg_.lethal_threshold);
  }

  // Per-cell traversal cost FACTOR for a passable cell.
  // Unknown cells (when allow_unknown=true, v<0) are treated as free (factor 1.0).
  // Inflation cells (0 <= v < lethal_threshold): factor = 1 + weight * (v / 100).
  // A* multiplies this factor by the geometric move distance to get edge cost.
  double stepCost(int8_t v) const {
    if (v < 0) return 1.0;
    return 1.0 + cfg_.inflation_cost_weight * (static_cast<double>(v) / 100.0);
  }

  const PlannerConfig& config() const { return cfg_; }

 private:
  PlannerConfig cfg_;
};

}  // namespace g1_global_planner
