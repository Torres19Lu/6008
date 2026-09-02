#pragma once

#include <cstdint>
#include <vector>

namespace g1_global_planner {

// Read-only view of a 2D OccupancyGrid slice used by the core planner.
// Row-major: index = my * width_ + mx. Cell (0,0) lower-left corner at
// world (origin_x_, origin_y_), matching nav_msgs/OccupancyGrid origin.
// Grid values are int8_t in the OccupancyGrid convention:
//   -1 = UNKNOWN, 0 = FREE, 1..98 = inflation, 99 = INSCRIBED, 100 = LETHAL.
class PlannerGrid {
 public:
  PlannerGrid() = default;

  // Construct from an existing flat data buffer (row-major, length width*height).
  PlannerGrid(double resolution, double origin_x, double origin_y,
              int width, int height, const std::vector<int8_t>& data);

  // Construct a sized grid filled with `fill`.
  PlannerGrid(double resolution, double origin_x, double origin_y,
              int width, int height, int8_t fill = 0);

  bool inBounds(int mx, int my) const {
    return mx >= 0 && mx < width_ && my >= 0 && my < height_;
  }

  int8_t at(int mx, int my) const { return data_[my * width_ + mx]; }
  void   setAt(int mx, int my, int8_t v) { data_[my * width_ + mx] = v; }

  // Map a world point to a cell index via floor((w - origin) / res).
  // Returns false (and leaves mx/my unchanged) when the result is out-of-bounds.
  bool worldToMap(double wx, double wy, int& mx, int& my) const;

  // Return the world coordinate of a cell's CENTER: origin + (m + 0.5) * res.
  void mapToWorld(int mx, int my, double& wx, double& wy) const;

  int    width()      const { return width_; }
  int    height()     const { return height_; }
  int    cells()      const { return static_cast<int>(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_)); }
  double resolution() const { return resolution_; }
  double originX()    const { return origin_x_; }
  double originY()    const { return origin_y_; }
  const std::vector<int8_t>& data() const { return data_; }

 private:
  double resolution_ = 0.0;
  double origin_x_   = 0.0;
  double origin_y_   = 0.0;
  int    width_      = 0;
  int    height_     = 0;
  std::vector<int8_t> data_;
};

}  // namespace g1_global_planner
