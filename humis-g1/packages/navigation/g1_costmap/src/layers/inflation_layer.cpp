#include "g1_costmap/layers/inflation_layer.h"

#include <cmath>
#include <queue>
#include <vector>

namespace g1_costmap {

namespace {
// Wavefront entry: the cell being visited and the obstacle source it came from,
// so the cell's cost uses the true Euclidean distance to that source.
struct Seed {
  int x, y;        // cell being assigned
  int sx, sy;      // nearest obstacle source cell
  double dist;     // Euclidean distance in cells (for the priority order)
};
struct Greater {
  bool operator()(const Seed& a, const Seed& b) const { return a.dist > b.dist; }
};
}  // namespace

void InflationLayer::inflate(CostmapGrid& grid, const InflationParams& p) const {
  if (!grid.initialized() || grid.resolution() <= 0.0) return;

  const double res = grid.resolution();
  const int nx = static_cast<int>(grid.sizeX());
  const int ny = static_cast<int>(grid.sizeY());
  const double cell_inflation = p.inflation_radius / res;  // radius in cells

  std::vector<char> seen(grid.cells(), 0);
  std::priority_queue<Seed, std::vector<Seed>, Greater> pq;

  // Seed the wavefront with every lethal cell (distance 0, source = itself).
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      if (grid.at(x, y) == LETHAL_OBSTACLE) {
        pq.push(Seed{x, y, x, y, 0.0});
      }
    }
  }

  while (!pq.empty()) {
    const Seed s = pq.top();
    pq.pop();
    const unsigned int idx = grid.index(s.x, s.y);
    if (seen[idx]) continue;
    seen[idx] = 1;

    // Assign cost from the Euclidean distance to the nearest obstacle source.
    if (s.dist > 0.0) {
      const double dist_m = s.dist * res;
      std::uint8_t cost;
      if (dist_m <= p.robot_radius) {
        cost = INSCRIBED_INFLATED;
      } else {
        const double factor =
            std::exp(-p.cost_scaling_factor * (dist_m - p.robot_radius));
        cost = static_cast<std::uint8_t>((INSCRIBED_INFLATED - 1) * factor);
      }
      // Never lower an existing higher cost (e.g. an overlapping lethal cell).
      if (cost > grid.at(s.x, s.y)) grid.setCost(s.x, s.y, cost);
    }

    // Expand to 8-neighbours, keeping the same obstacle source.
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;
        const int nxc = s.x + dx;
        const int nyc = s.y + dy;
        if (nxc < 0 || nyc < 0 || nxc >= nx || nyc >= ny) continue;
        if (seen[grid.index(nxc, nyc)]) continue;
        const double ndist =
            std::hypot(static_cast<double>(nxc - s.sx),
                       static_cast<double>(nyc - s.sy));
        if (ndist > cell_inflation) continue;
        pq.push(Seed{nxc, nyc, s.sx, s.sy, ndist});
      }
    }
  }
}

}  // namespace g1_costmap
