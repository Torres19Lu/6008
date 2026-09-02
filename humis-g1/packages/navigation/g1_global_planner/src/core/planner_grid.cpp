#include "g1_global_planner/core/planner_grid.h"

#include <cmath>
#include <stdexcept>

namespace g1_global_planner {

PlannerGrid::PlannerGrid(double resolution, double origin_x, double origin_y,
                         int width, int height,
                         const std::vector<int8_t>& data)
    : resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      width_(width),
      height_(height),
      data_(data) {
  if (data.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
    throw std::invalid_argument("PlannerGrid: data size does not match width*height");
  }
}

PlannerGrid::PlannerGrid(double resolution, double origin_x, double origin_y,
                         int width, int height, int8_t fill)
    : resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      width_(width),
      height_(height),
      data_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), fill) {}

bool PlannerGrid::worldToMap(double wx, double wy, int& mx, int& my) const {
  int cx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  int cy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  if (!inBounds(cx, cy)) return false;
  mx = cx;
  my = cy;
  return true;
}

void PlannerGrid::mapToWorld(int mx, int my, double& wx, double& wy) const {
  wx = origin_x_ + (static_cast<double>(mx) + 0.5) * resolution_;
  wy = origin_y_ + (static_cast<double>(my) + 0.5) * resolution_;
}

}  // namespace g1_global_planner
