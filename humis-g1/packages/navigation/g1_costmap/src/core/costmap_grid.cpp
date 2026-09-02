#include "g1_costmap/core/costmap_grid.h"

#include <algorithm>
#include <cmath>

namespace g1_costmap {

CostmapGrid::CostmapGrid(double resolution, double origin_x, double origin_y,
                         unsigned int size_x, unsigned int size_y,
                         std::uint8_t fill) {
  resize(resolution, origin_x, origin_y, size_x, size_y, fill);
}

void CostmapGrid::resize(double resolution, double origin_x, double origin_y,
                         unsigned int size_x, unsigned int size_y,
                         std::uint8_t fill) {
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  size_x_ = size_x;
  size_y_ = size_y;
  cost_.assign(static_cast<std::size_t>(size_x) * size_y, fill);
}

void CostmapGrid::reset(std::uint8_t fill) {
  std::fill(cost_.begin(), cost_.end(), fill);
}

bool CostmapGrid::worldToMap(double wx, double wy, unsigned int& mx,
                             unsigned int& my) const {
  if (resolution_ <= 0.0 || wx < origin_x_ || wy < origin_y_) return false;
  const auto x = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  const auto y = static_cast<unsigned int>((wy - origin_y_) / resolution_);
  if (x < size_x_ && y < size_y_) {
    mx = x;
    my = y;
    return true;
  }
  return false;
}

void CostmapGrid::mapToWorld(unsigned int mx, unsigned int my, double& wx,
                             double& wy) const {
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

}  // namespace g1_costmap
