#pragma once

// costmap_view.h -- ROS-free local costmap snapshot for g1_local_planner.
// No ROS headers. No Eigen. std-only.
//
// Holds a copy of the /nav/local_costmap OccupancyGrid data. The node (Task 6)
// constructs a CostmapView from the received message and hands it to the core.
// Cell index convention (matching nav_msgs/OccupancyGrid):
//   index = y * width + x    (row-major, y=0 at the bottom of the grid)
//   cell value: -1 = unknown, 0 = free, 1..99 = inflation, 100 = lethal/inscribed
// Cell CENTRE in world (map) frame:
//   wx = origin_x + (mx + 0.5) * resolution
//   wy = origin_y + (my + 0.5) * resolution

#include <cstdint>
#include <utility>
#include <vector>

namespace g1_local_planner {

class CostmapView {
 public:
  // Default ctor: empty/invalid (cells() == 0, valid() == false).
  CostmapView() = default;

  // Construct from a data buffer (row-major, length == width * height).
  // lethal_cost and treat_unknown_as_obstacle are copied from MpcConfig at
  // the call site so the core stays independent of the config type.
  CostmapView(double resolution,
              double origin_x,
              double origin_y,
              int    width,
              int    height,
              const std::vector<int8_t>& data,
              double lethal_cost,
              bool   treat_unknown_as_obstacle);

  // True when the grid holds at least one cell.
  bool   valid() const { return cells() > 0; }
  size_t cells() const { return data_.size(); }

  int    width()      const { return width_; }
  int    height()     const { return height_; }
  double resolution() const { return resolution_; }
  double originX()    const { return origin_x_; }
  double originY()    const { return origin_y_; }

  // Map a world point to integer cell indices via floor.
  // Returns false (and leaves mx/my unchanged) when outside [0,width) x [0,height).
  bool worldToMap(double wx, double wy, int& mx, int& my) const;

  // True when (mx, my) is inside the grid.
  bool inBounds(int mx, int my) const {
    return mx >= 0 && mx < width_ && my >= 0 && my < height_;
  }

  // Raw OccupancyGrid value at cell (mx, my). Caller must ensure inBounds.
  int8_t at(int mx, int my) const { return data_[my * width_ + mx]; }

  // Normalize a raw cell value to [0, lethal_cost]:
  //   0   -> 0.0
  //   1..99 -> v / 100.0
  //   100 -> lethal_cost
  //   -1  -> treat_unknown_as_obstacle ? lethal_cost : 0.0
  double normCost(int8_t v) const;

  // Bilinear interpolation of normCost over the 4 surrounding cell centres at
  // world point (wx, wy). Out-of-bounds returns lethal_cost (hard penalty).
  // The bilinear sample cells are clamped to the grid boundary so a point near
  // an edge still returns a valid finite value.
  double cost(double wx, double wy) const;

  // Central finite difference of cost() over +- one cell (step = resolution),
  // returning (gx, gy) in cost-per-metre. The gradient points toward INCREASING
  // cost (i.e. toward obstacles); the MPC pushes DOWN-gradient to avoid them.
  std::pair<double, double> gradient(double wx, double wy) const;

 private:
  double resolution_               = 0.0;
  double origin_x_                 = 0.0;
  double origin_y_                 = 0.0;
  int    width_                    = 0;
  int    height_                   = 0;
  double lethal_cost_              = 1.0;
  bool   treat_unknown_as_obstacle_ = true;
  std::vector<int8_t> data_;
};

}  // namespace g1_local_planner
