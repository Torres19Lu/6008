#pragma once

// cropCentered: produce a robot-centered crop of the fused inflated master for
// the local MPC window. This is a VIEW of the master grid: cells
// inside the master bounds are copied; cells that fall outside are padded as
// NO_INFORMATION (unknown). The output is published by the node as
// /nav/local_costmap (nav_msgs/OccupancyGrid, map frame) so that g1_local_planner
// can query only a small window around the robot without reading the full global
// grid. ROS-free; std-only.

#include "g1_costmap/core/costmap_grid.h"

namespace g1_costmap {

// Return a square CostmapGrid of size (n x n) cells where
//   n = std::lround(size_m / src.resolution())
// centred on the src cell that contains world point (center_x, center_y).
//
// Window alignment:
//   center_cell_x = floor((center_x - src.originX()) / res)
//   center_cell_y = floor((center_y - src.originY()) / res)
//   lower-left src-cell index: ll_x = center_cell_x - n/2  (integer division)
//                              ll_y = center_cell_y - n/2
//   new origin: src.originX() + ll_x * res  (aligns to the src grid)
//
// For each window cell (i, j) the corresponding src cell is (ll_x+i, ll_y+j).
// If that src cell is inside the src bounds, the value is copied from src.data();
// otherwise the window cell is set to NO_INFORMATION (255).
//
// Returns an empty CostmapGrid (cells() == 0) when:
//   - src.cells() == 0  (empty master)
//   - size_m <= 0
//   - src.resolution() <= 0
CostmapGrid cropCentered(const CostmapGrid& src, double center_x,
                         double center_y, double size_m);

}  // namespace g1_costmap
