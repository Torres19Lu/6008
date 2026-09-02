#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace g1_costmap {

// Persistent per-cell ground-height surface in the map frame. There is no
// pre-segmented ground source, so each (x,y) cell first takes the LOWEST
// observed z (the
// floor is the lowest return in a cell). The reported ground at a cell is then
// the MEDIAN of the trusted cell-lowests within `max_fill_radius` (including the
// cell itself): the median rejects the isolated far-below-floor outliers that a
// raw SLAM cloud carries, fills sparse single-point cells from their neighbours,
// and smooths gentle drift, while staying slope-aware (each cell against its own
// local neighbourhood). A cell is trusted once it holds `min_points` returns; the
// robust median is precomputed in update() so groundZAt is O(1). groundZAt
// returns nullopt for a cell with no trusted neighbourhood (the caller falls back
// to the foot-contact scalar).
class GroundSurface {
 public:
  GroundSurface() = default;
  GroundSurface(double cell_size, double max_fill_radius, int min_points) {
    configure(cell_size, max_fill_radius, min_points);
  }
  void configure(double cell_size, double max_fill_radius, int min_points);

  // Integrate map-frame points (per cell: lowest z, count), then precompute the
  // robust median ground for every trusted cell.
  void update(const std::vector<Eigen::Vector3f>& points_map);
  void clear() {
    cells_.clear();
    robust_.clear();
  }

  std::optional<double> groundZAt(double x, double y) const;
  std::size_t cellCount() const { return robust_.size(); }
  double cellSize() const { return cell_size_; }

 private:
  struct Acc {
    double low = 0.0;
    int count = 0;
  };
  static std::int64_t key(int ix, int iy);
  int cellIndex(double v) const;

  double cell_size_ = 0.1;
  double max_fill_radius_ = 0.3;
  int min_points_ = 1;
  std::unordered_map<std::int64_t, Acc> cells_;      // raw lowest + count
  std::unordered_map<std::int64_t, double> robust_;  // median ground per cell
};

// Ground height at a query point: prefer the per-cell GroundSurface, fall back to
// the foot-contact scalar. `valid` is false only when neither is available.
struct GroundLookup {
  const GroundSurface* surface = nullptr;
  double foot_scalar = 0.0;
  bool has_foot = false;

  bool valid(double x, double y) const {
    if (surface && surface->groundZAt(x, y)) return true;
    return has_foot;
  }
  double at(double x, double y) const {
    if (surface) {
      const auto g = surface->groundZAt(x, y);
      if (g) return *g;
    }
    return foot_scalar;
  }
};

}  // namespace g1_costmap
