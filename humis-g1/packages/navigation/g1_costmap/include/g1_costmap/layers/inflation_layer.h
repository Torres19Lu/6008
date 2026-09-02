#pragma once

#include "g1_costmap/core/costmap_grid.h"

namespace g1_costmap {

struct InflationParams {
  double robot_radius = 0.3;        // m; inscribed radius (cells within -> INSCRIBED)
  double inflation_radius = 0.8;    // m; cost decays to free at this distance
  double cost_scaling_factor = 3.0; // exponential falloff rate (per m past inscribed)
};

// Inflates LETHAL cells in-place: cells within `robot_radius` of an obstacle
// become INSCRIBED_INFLATED, and cells out to `inflation_radius` get an
// exponentially decaying cost. Distances are true Euclidean (a cell tracks its
// nearest obstacle source as the wavefront propagates).
class InflationLayer {
 public:
  void inflate(CostmapGrid& grid, const InflationParams& p) const;
};

}  // namespace g1_costmap
