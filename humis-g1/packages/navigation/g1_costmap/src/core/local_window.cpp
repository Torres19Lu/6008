// cropCentered: robot-centered crop of the fused inflated master for the local
// MPC window. See local_window.h for the full semantics.

#include "g1_costmap/core/local_window.h"

#include <cmath>
#include <cstdint>

#include "g1_costmap/core/cost_values.h"

namespace g1_costmap {

CostmapGrid cropCentered(const CostmapGrid& src, double center_x,
                         double center_y, double size_m) {
  if (src.cells() == 0 || size_m <= 0.0 || src.resolution() <= 0.0) {
    return CostmapGrid{};
  }

  const double res = src.resolution();
  // Number of cells along each side of the square window.
  const int n = static_cast<int>(std::lround(size_m / res));
  if (n <= 0) {
    return CostmapGrid{};
  }

  // Src cell containing the crop center (floor division).
  const int cx =
      static_cast<int>(std::floor((center_x - src.originX()) / res));
  const int cy =
      static_cast<int>(std::floor((center_y - src.originY()) / res));

  // Lower-left src-cell index of the window.
  const int ll_x = cx - n / 2;
  const int ll_y = cy - n / 2;

  // New origin aligns the window to the src grid.
  const double new_origin_x = src.originX() + static_cast<double>(ll_x) * res;
  const double new_origin_y = src.originY() + static_cast<double>(ll_y) * res;

  CostmapGrid out(res, new_origin_x, new_origin_y,
                  static_cast<unsigned int>(n), static_cast<unsigned int>(n),
                  NO_INFORMATION);

  const int src_sx = static_cast<int>(src.sizeX());
  const int src_sy = static_cast<int>(src.sizeY());

  for (int j = 0; j < n; ++j) {
    const int sy = ll_y + j;
    if (sy < 0 || sy >= src_sy) continue;  // out of src bounds -> keep NO_INFORMATION
    for (int i = 0; i < n; ++i) {
      const int sx = ll_x + i;
      if (sx < 0 || sx >= src_sx) continue;  // out of src bounds -> keep NO_INFORMATION
      const std::uint8_t v =
          src.data()[src.index(static_cast<unsigned int>(sx),
                               static_cast<unsigned int>(sy))];
      out.setCost(static_cast<unsigned int>(i), static_cast<unsigned int>(j), v);
    }
  }

  return out;
}

}  // namespace g1_costmap
