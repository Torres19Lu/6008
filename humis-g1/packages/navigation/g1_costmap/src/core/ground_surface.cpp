#include "g1_costmap/core/ground_surface.h"

#include <algorithm>
#include <cmath>

namespace g1_costmap {

void GroundSurface::configure(double cell_size, double max_fill_radius,
                              int min_points) {
  cell_size_ = cell_size > 0.0 ? cell_size : 0.1;
  max_fill_radius_ = max_fill_radius >= 0.0 ? max_fill_radius : 0.0;
  min_points_ = std::max(1, min_points);
  clear();
}

std::int64_t GroundSurface::key(int ix, int iy) {
  const std::uint64_t hi = static_cast<std::uint64_t>(
                               static_cast<std::uint32_t>(ix))
                           << 32;
  const std::uint64_t lo =
      static_cast<std::uint64_t>(static_cast<std::uint32_t>(iy));
  return static_cast<std::int64_t>(hi | lo);
}

int GroundSurface::cellIndex(double v) const {
  return static_cast<int>(std::floor(v / cell_size_));
}

void GroundSurface::update(const std::vector<Eigen::Vector3f>& points_map) {
  for (const auto& p : points_map) {
    const std::int64_t k = key(cellIndex(p.x()), cellIndex(p.y()));
    Acc& a = cells_[k];
    if (a.count == 0 || p.z() < a.low) a.low = p.z();
    ++a.count;
  }

  // Precompute the robust median ground for every trusted cell.
  robust_.clear();
  const int r = static_cast<int>(std::ceil(max_fill_radius_ / cell_size_));
  const double r2 = max_fill_radius_ * max_fill_radius_;
  std::vector<float> nbr;
  for (const auto& kv : cells_) {
    if (kv.second.count < min_points_) continue;
    const int ix = static_cast<int>(static_cast<std::int32_t>(
        static_cast<std::uint32_t>(static_cast<std::uint64_t>(kv.first) >> 32)));
    const int iy = static_cast<int>(static_cast<std::int32_t>(
        static_cast<std::uint32_t>(static_cast<std::uint64_t>(kv.first))));
    nbr.clear();
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if ((dx * dx + dy * dy) * cell_size_ * cell_size_ > r2) continue;
        const auto n = cells_.find(key(ix + dx, iy + dy));
        if (n == cells_.end() || n->second.count < min_points_) continue;
        nbr.push_back(static_cast<float>(n->second.low));
      }
    }
    if (nbr.empty()) continue;  // cannot happen (self qualifies), defensive
    const std::size_t mid = nbr.size() / 2;
    std::nth_element(nbr.begin(), nbr.begin() + mid, nbr.end());
    const double hi = nbr[mid];
    if (nbr.size() % 2 == 1) {
      robust_[kv.first] = hi;  // odd: the middle element is the median
    } else {
      // Even count: average the two central elements (the lower middle is the max
      // of the left partition) so the ground is not biased upward.
      const double lo = *std::max_element(nbr.begin(), nbr.begin() + mid);
      robust_[kv.first] = 0.5 * (lo + hi);
    }
  }
}

std::optional<double> GroundSurface::groundZAt(double x, double y) const {
  const auto it = robust_.find(key(cellIndex(x), cellIndex(y)));
  if (it != robust_.end()) return it->second;
  return std::nullopt;
}

}  // namespace g1_costmap
