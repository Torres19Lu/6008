#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "g1_costmap/core/cost_values.h"

namespace g1_costmap {

// A 2D occupancy cost grid in a fixed metric frame (the costmap is built in the
// `map` frame). Row-major, cell (0,0) at world (origin_x, origin_y). ROS-free:
// the node converts to/from nav_msgs/OccupancyGrid.
class CostmapGrid {
 public:
  CostmapGrid() = default;
  CostmapGrid(double resolution, double origin_x, double origin_y,
              unsigned int size_x, unsigned int size_y,
              std::uint8_t fill = FREE_SPACE);

  // (Re)allocate to the given geometry, filling every cell with `fill`.
  void resize(double resolution, double origin_x, double origin_y,
              unsigned int size_x, unsigned int size_y,
              std::uint8_t fill = FREE_SPACE);

  bool initialized() const { return size_x_ > 0 && size_y_ > 0; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  unsigned int sizeX() const { return size_x_; }
  unsigned int sizeY() const { return size_y_; }
  std::size_t cells() const { return cost_.size(); }

  unsigned int index(unsigned int mx, unsigned int my) const {
    return my * size_x_ + mx;
  }
  void indexToCells(unsigned int idx, unsigned int& mx, unsigned int& my) const {
    my = idx / size_x_;
    mx = idx - my * size_x_;
  }

  // World (metric) <-> cell. worldToMap returns false if the point is outside
  // the grid; mapToWorld returns the cell CENTER.
  bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
  void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

  std::uint8_t at(unsigned int mx, unsigned int my) const {
    return cost_[index(mx, my)];
  }
  void setCost(unsigned int mx, unsigned int my, std::uint8_t c) {
    cost_[index(mx, my)] = c;
  }

  const std::vector<std::uint8_t>& data() const { return cost_; }
  std::vector<std::uint8_t>& data() { return cost_; }
  void reset(std::uint8_t fill = FREE_SPACE);

 private:
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  unsigned int size_x_ = 0;
  unsigned int size_y_ = 0;
  std::vector<std::uint8_t> cost_;
};

}  // namespace g1_costmap
