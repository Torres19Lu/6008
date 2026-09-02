#include "g1_local_planner/core/costmap_view.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_local_planner {

CostmapView::CostmapView(double resolution,
                         double origin_x,
                         double origin_y,
                         int    width,
                         int    height,
                         const std::vector<int8_t>& data,
                         double lethal_cost,
                         bool   treat_unknown_as_obstacle)
    : resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      width_(width),
      height_(height),
      lethal_cost_(lethal_cost),
      treat_unknown_as_obstacle_(treat_unknown_as_obstacle),
      data_(data) {
  if (data.size() != static_cast<size_t>(width) * static_cast<size_t>(height)) {
    throw std::invalid_argument(
        "CostmapView: data size does not match width * height");
  }
}

bool CostmapView::worldToMap(double wx, double wy, int& mx, int& my) const {
  int cx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  int cy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  if (!inBounds(cx, cy)) return false;
  mx = cx;
  my = cy;
  return true;
}

double CostmapView::normCost(int8_t v) const {
  if (v == 0)   return 0.0;
  if (v >= 100) return lethal_cost_;   // 100 lethal/inscribed; >100 is off-spec, saturate
  if (v < 0)    return treat_unknown_as_obstacle_ ? lethal_cost_ : 0.0;
  // v in 1..99
  return static_cast<double>(v) / 100.0;
}

// Bilinear interpolation of normCost over the 4 surrounding cell centres.
// Cell centre of (mx, my) is at (origin + (mx+0.5)*res, origin + (my+0.5)*res).
// We need the 4 cells whose centres bracket (wx, wy).
//   The lower-left bracket cell (x0, y0) satisfies:
//     origin + (x0 + 0.5) * res <= wx < origin + (x0 + 1.5) * res
//   Solving: x0 = floor((wx - origin)/res - 0.5)
// We then sample at (x0, y0), (x0+1, y0), (x0, y0+1), (x0+1, y0+1),
// clamping each to [0, width-1] x [0, height-1].
double CostmapView::cost(double wx, double wy) const {
  if (!valid()) return lethal_cost_;

  // Check if point is fully outside the grid extents (world coords of grid).
  double grid_max_x = origin_x_ + width_  * resolution_;
  double grid_max_y = origin_y_ + height_ * resolution_;
  if (wx < origin_x_ || wx >= grid_max_x ||
      wy < origin_y_ || wy >= grid_max_y) {
    return lethal_cost_;
  }

  // Fractional cell coordinate relative to cell-centre grid.
  // cx_f = 0.0 corresponds to the centre of cell x=0 (at origin + 0.5*res).
  double cx_f = (wx - origin_x_) / resolution_ - 0.5;
  double cy_f = (wy - origin_y_) / resolution_ - 0.5;

  // Lower-left bracket cell, clamped.
  int x0 = static_cast<int>(std::floor(cx_f));
  int y0 = static_cast<int>(std::floor(cy_f));

  // Fractional offsets within [0, 1] for bilinear weights.
  double tx = cx_f - static_cast<double>(x0);
  double ty = cy_f - static_cast<double>(y0);

  // Clamp bracket indices to valid cell range.
  int x0c = std::max(0, std::min(x0,     width_  - 1));
  int x1c = std::max(0, std::min(x0 + 1, width_  - 1));
  int y0c = std::max(0, std::min(y0,     height_ - 1));
  int y1c = std::max(0, std::min(y0 + 1, height_ - 1));

  double c00 = normCost(at(x0c, y0c));
  double c10 = normCost(at(x1c, y0c));
  double c01 = normCost(at(x0c, y1c));
  double c11 = normCost(at(x1c, y1c));

  // Standard bilinear interpolation.
  return (1.0 - tx) * (1.0 - ty) * c00
       + tx         * (1.0 - ty) * c10
       + (1.0 - tx) * ty         * c01
       + tx         * ty         * c11;
}

// Central finite difference of cost() with step = resolution.
// Returns (gx, gy) in cost-per-metre (positive = toward higher cost = toward obstacles).
std::pair<double, double> CostmapView::gradient(double wx, double wy) const {
  double step = resolution_;
  double gx = (cost(wx + step, wy) - cost(wx - step, wy)) / (2.0 * step);
  double gy = (cost(wx, wy + step) - cost(wx, wy - step)) / (2.0 * step);
  return {gx, gy};
}

}  // namespace g1_local_planner
