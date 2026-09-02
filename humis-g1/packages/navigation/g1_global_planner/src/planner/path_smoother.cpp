#include "g1_global_planner/planner/path_smoother.h"

#include <cmath>
#include <cstdlib>

namespace g1_global_planner {

// Amanatides-Woo DDA supercover traversal with exact integer arithmetic.
//
// Visits every grid cell the straight segment between the centers of cells a
// and b passes through. Conservative at exact corner crossings: when the
// segment passes exactly through a grid corner, both edge-adjacent cells are
// checked. Returns false as soon as any visited cell is out-of-bounds or
// blocked. Both endpoint cells are included in the check.
//
// The parametric variable t spans [0, 1] from center(a) to center(b). All t
// comparisons are scaled by 2*|dx|*|dy| and kept as long long integers to
// avoid floating-point drift.
//   tDeltaX = 2*|dy|   (parametric cost to cross one cell in X)
//   tDeltaY = 2*|dx|   (parametric cost to cross one cell in Y)
//   tMaxX   = |dy|     (distance from center of a to first vertical boundary)
//   tMaxY   = |dx|     (distance from center of a to first horizontal boundary)
bool lineOfSight(const PlannerGrid& grid,
                 const CostModel&   cost,
                 Cell a,
                 Cell b) {
  if (!grid.inBounds(a.x, a.y) || cost.blocked(grid.at(a.x, a.y))) return false;
  if (a.x == b.x && a.y == b.y) return true;

  const int dx_abs = std::abs(b.x - a.x);
  const int dy_abs = std::abs(b.y - a.y);
  const int stepX  = (b.x > a.x) ? 1 : (b.x < a.x) ? -1 : 0;
  const int stepY  = (b.y > a.y) ? 1 : (b.y < a.y) ? -1 : 0;

  // A large sentinel value that is never reached in practice.
  // When stepping only along one axis the other tMax is never smaller.
  const long long INF = static_cast<long long>(dx_abs + dy_abs + 1) * 2 + 1;

  const long long tDeltaX = (dx_abs > 0) ? static_cast<long long>(2) * dy_abs : INF;
  const long long tDeltaY = (dy_abs > 0) ? static_cast<long long>(2) * dx_abs : INF;
  long long tMaxX          = (dx_abs > 0) ? static_cast<long long>(dy_abs) : INF;
  long long tMaxY          = (dy_abs > 0) ? static_cast<long long>(dx_abs) : INF;

  int x = a.x;
  int y = a.y;

  // Maximum crossings: |dx| vertical + |dy| horizontal boundaries.
  const int max_steps = dx_abs + dy_abs + 2;

  for (int step = 0; step < max_steps; ++step) {
    if (x == b.x && y == b.y) return true;

    if (tMaxX < tMaxY) {
      x += stepX;
      tMaxX += tDeltaX;
    } else if (tMaxY < tMaxX) {
      y += stepY;
      tMaxY += tDeltaY;
    } else {
      // Exact corner crossing: segment passes through a grid corner. Check
      // both edge-adjacent cells conservatively before advancing diagonally.
      const int nx = x + stepX;
      const int ny = y + stepY;
      if (!grid.inBounds(nx, y) || cost.blocked(grid.at(nx, y))) return false;
      if (!grid.inBounds(x, ny) || cost.blocked(grid.at(x, ny))) return false;
      x    += stepX;
      y    += stepY;
      tMaxX += tDeltaX;
      tMaxY += tDeltaY;
    }

    if (!grid.inBounds(x, y) || cost.blocked(grid.at(x, y))) return false;
  }

  return true;
}

std::vector<Cell> smoothPath(const PlannerGrid& grid,
                             const CostModel&   cost,
                             const PlannerConfig& cfg,
                             const std::vector<Cell>& cells) {
  if (!cfg.smooth_enable || cells.size() < 3) return cells;

  std::vector<Cell> result;
  result.push_back(cells.front());

  std::size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    // Find the farthest index reachable from anchor with a clear line of sight.
    std::size_t farthest = anchor + 1;
    for (std::size_t j = anchor + 2; j < cells.size(); ++j) {
      if (lineOfSight(grid, cost, cells[anchor], cells[j])) {
        farthest = j;
      }
    }
    result.push_back(cells[farthest]);
    anchor = farthest;
  }

  return result;
}

}  // namespace g1_global_planner
