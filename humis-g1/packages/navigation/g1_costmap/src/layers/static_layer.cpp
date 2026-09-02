#include "g1_costmap/layers/static_layer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "g1_costmap/core/ground_classifier.h"

namespace g1_costmap {
namespace {

// Robust XY bounds (percentile) so far-flung outliers do not blow the grid up.
void robustBounds(std::vector<float>& xs, std::vector<float>& ys, double clip_pct,
                  double& min_x, double& min_y, double& max_x, double& max_y) {
  const double q = std::min(0.49, std::max(0.0, clip_pct));
  auto pctl = [](std::vector<float>& v, double frac) {
    const std::size_t i = static_cast<std::size_t>(frac * (v.size() - 1));
    std::nth_element(v.begin(), v.begin() + i, v.end());
    return static_cast<double>(v[i]);
  };
  min_x = pctl(xs, q);
  max_x = pctl(xs, 1.0 - q);
  min_y = pctl(ys, q);
  max_y = pctl(ys, 1.0 - q);
}

void sizeGrid(double min_x, double min_y, double max_x, double max_y,
              double resolution, double margin, CostmapGrid& out) {
  const double origin_x = min_x - margin;
  const double origin_y = min_y - margin;
  const double span_x = (max_x - min_x) + 2.0 * margin;
  const double span_y = (max_y - min_y) + 2.0 * margin;
  const auto size_x =
      static_cast<unsigned int>(std::ceil(span_x / resolution)) + 1;
  const auto size_y =
      static_cast<unsigned int>(std::ceil(span_y / resolution)) + 1;
  out.resize(resolution, origin_x, origin_y, size_x, size_y, NO_INFORMATION);
}

// Morphological close: fill UNKNOWN cells enclosed by FREE (>= 5 of 8 free
// neighbours) to FREE, closing the voxel-sparse holes without bulging into open
// unknown space. Obstacle cells are never overwritten.
void morphClose(CostmapGrid& g, int iters) {
  const int nx = static_cast<int>(g.sizeX());
  const int ny = static_cast<int>(g.sizeY());
  for (int it = 0; it < iters; ++it) {
    std::vector<unsigned int> to_free;
    for (int y = 0; y < ny; ++y) {
      for (int x = 0; x < nx; ++x) {
        if (g.at(x, y) != NO_INFORMATION) continue;
        int free_n = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int xx = x + dx, yy = y + dy;
            if (xx < 0 || yy < 0 || xx >= nx || yy >= ny) continue;
            if (g.at(xx, yy) == FREE_SPACE) ++free_n;
          }
        }
        if (free_n >= 5) to_free.push_back(g.index(x, y));
      }
    }
    if (to_free.empty()) break;
    for (const unsigned int idx : to_free) g.data()[idx] = FREE_SPACE;
  }
}

// Classify a point's height against the cell ground; mark the cell (bare
// projection path, last-write). Returns the Tier3 for the caller.
Tier3 markPoint(const Eigen::Vector3f& pt, const GroundLookup& ground,
                const StaticLayerParams& p, CostmapGrid& out, unsigned int mx,
                unsigned int my) {
  const double gz = ground.at(pt.x(), pt.y());
  const Tier3 tier = classify3(
      layerOf(pt.z(), p.resolution) - layerOf(gz, p.resolution),
      p.walkable_layers, p.obstacle_max_layers);
  if (tier == Tier3::OBSTACLE) {
    out.setCost(mx, my, LETHAL_OBSTACLE);  // obstacle wins (unconditional)
  } else if (tier == Tier3::FREE) {
    if (out.at(mx, my) == NO_INFORMATION) out.setCost(mx, my, FREE_SPACE);
  }
  return tier;
}

// Per-keyframe local grid -> log-odds accumulator. For one keyframe it segments
// each return (height band + optional normal gate), raycasts the beam path to
// FREE, angular-fills the unobserved wedges in front of the observed surfaces,
// resolves OBSTACLE-over-FREE per cell, and casts ONE hit/miss vote per touched
// cell into the shared accumulator (so one keyframe cannot double-count a cell).
class LocalGridMaker {
 public:
  LocalGridMaker(const CostmapGrid& geom, const StaticLayerParams& p,
                 const GroundLookup& ground, OccupancyAccumulator& acc)
      : geom_(geom),
        p_(p),
        ground_(ground),
        acc_(acc),
        nx_(static_cast<int>(geom.sizeX())),
        ny_(static_cast<int>(geom.sizeY())),
        stamp_(geom.cells(), -1),
        state_(geom.cells(), kUnseen) {
    if (p_.angular_fill) {
      const double step = p_.angular_fill_step > 1e-4 ? p_.angular_fill_step : 0.0087;
      nbins_ = std::max(1, static_cast<int>(std::ceil(2.0 * M_PI / step)));
      step_ = 2.0 * M_PI / nbins_;
    }
  }

  void integrate(const StaticKeyframe& kf, int kf_id) {
    lidar_z_ = kf.origin.z();  // band TOP follows this keyframe's lidar plane
    std::vector<unsigned int> touched;
    const int ox = cellX(kf.origin.x());
    const int oy = cellY(kf.origin.y());
    const double max_range_sq = p_.max_range > 0.0
                                    ? p_.max_range * p_.max_range
                                    : std::numeric_limits<double>::infinity();
    const bool have_normals =
        p_.use_normals && kf.normals.size() == kf.points.size();

    std::vector<double> bin_obs_min, bin_free_max;
    if (p_.angular_fill) {
      bin_obs_min.assign(nbins_, std::numeric_limits<double>::infinity());
      bin_free_max.assign(nbins_, 0.0);
    }

    // Pass 1: classify every return and mark the OBSTACLE cells first, so the free
    // ray tracing in pass 2 can stop at them (a beam passing over an obstacle in
    // 3D must not clear the obstacle's footprint or anything occluded behind it).
    struct Hit {
      int mx, my;
      bool obstacle;
    };
    std::vector<Hit> hits;
    hits.reserve(kf.points.size());
    unsigned int mx = 0, my = 0;
    for (std::size_t i = 0; i < kf.points.size(); ++i) {
      const Eigen::Vector3f& pt = kf.points[i];
      const double ddx = pt.x() - kf.origin.x();
      const double ddy = pt.y() - kf.origin.y();
      const double rsq = ddx * ddx + ddy * ddy;
      if (rsq > max_range_sq) continue;
      if (!geom_.worldToMap(pt.x(), pt.y(), mx, my)) continue;
      if (!ground_.valid(pt.x(), pt.y())) continue;
      const Eigen::Vector3f* nrm = have_normals ? &kf.normals[i] : nullptr;
      const Tier3 tier = classifyReturn(pt, nrm);
      if (tier == Tier3::IGNORED) continue;  // ceiling/ghost: not reliable in 2D
      const bool obs = tier == Tier3::OBSTACLE;
      hits.push_back({static_cast<int>(mx), static_cast<int>(my), obs});
      if (obs) mark(static_cast<int>(mx), static_cast<int>(my), kObstacle, kf_id,
                    touched);
      if (p_.angular_fill) {
        const int b = binOf(std::atan2(ddy, ddx));
        const double r = std::sqrt(rsq);
        if (obs) {
          bin_obs_min[b] = std::min(bin_obs_min[b], r);
        } else {
          bin_free_max[b] = std::max(bin_free_max[b], r);
        }
      }
    }

    // Pass 2: clear the beam path to each return FREE, stopping at the first
    // obstacle; then mark the free-return endpoints (obstacles already marked).
    for (const Hit& h : hits) {
      rayFreeStop(ox, oy, h.mx, h.my, kf_id, touched);
      if (!h.obstacle) mark(h.mx, h.my, kFree, kf_id, touched);
    }

    if (p_.angular_fill) {
      for (int b = 0; b < nbins_; ++b) {
        const double limit =
            std::isfinite(bin_obs_min[b]) ? bin_obs_min[b] : bin_free_max[b];
        if (limit <= 0.0) continue;
        const double ang = -M_PI + (b + 0.5) * step_;
        const double ex = kf.origin.x() + std::cos(ang) * limit;
        const double ey = kf.origin.y() + std::sin(ang) * limit;
        fillFreeUntilObstacle(ox, oy, cellX(ex), cellY(ey), kf_id, touched);
      }
    }

    for (const unsigned int idx : touched) {
      if (state_[idx] == kObstacle) {
        acc_.observeHit(idx);
      } else if (state_[idx] == kFree) {
        acc_.observeMiss(idx);
      }
    }
  }

 private:
  enum : std::uint8_t { kUnseen = 0, kFree = 1, kObstacle = 2 };

  bool inGrid(int x, int y) const {
    return x >= 0 && y >= 0 && x < nx_ && y < ny_;
  }
  int cellX(double wx) const {
    return static_cast<int>(std::floor((wx - geom_.originX()) / p_.resolution));
  }
  int cellY(double wy) const {
    return static_cast<int>(std::floor((wy - geom_.originY()) / p_.resolution));
  }
  int binOf(double ang) const {
    int b = static_cast<int>(std::floor((ang + M_PI) / step_));
    return std::min(nbins_ - 1, std::max(0, b));
  }

  void mark(int x, int y, std::uint8_t s, int kf_id,
            std::vector<unsigned int>& touched) {
    if (!inGrid(x, y)) return;
    const unsigned int idx = geom_.index(x, y);
    if (stamp_[idx] != kf_id) {
      stamp_[idx] = kf_id;
      state_[idx] = s;
      touched.push_back(idx);
    } else if (s == kObstacle) {
      state_[idx] = kObstacle;  // obstacle wins; free never downgrades it
    }
  }

  // Bresenham origin->(x1,y1); mark each traversed cell EXCEPT the endpoint kFree,
  // but STOP at the first cell already OBSTACLE in this keyframe. This is the
  // occlusion guard: a beam that in 3D passes over an obstacle (its 2D footprint
  // crossing the obstacle) must not clear the obstacle or the cells behind it.
  void rayFreeStop(int x0, int y0, int x1, int y1, int kf_id,
                   std::vector<unsigned int>& touched) {
    const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy, x = x0, y = y0;
    while (x != x1 || y != y1) {
      if (inGrid(x, y)) {
        const unsigned int idx = geom_.index(x, y);
        if (stamp_[idx] == kf_id && state_[idx] == kObstacle) break;  // occluded
        if (stamp_[idx] != kf_id) {
          stamp_[idx] = kf_id;
          state_[idx] = kFree;
          touched.push_back(idx);
        }
      }
      const int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 < dx) { err += dx; y += sy; }
    }
  }

  // Bresenham origin->(x1,y1); mark UNSEEN cells kFree, stopping at the first cell
  // already OBSTACLE in this keyframe (never fill behind an obstacle).
  void fillFreeUntilObstacle(int x0, int y0, int x1, int y1, int kf_id,
                             std::vector<unsigned int>& touched) {
    const int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy, x = x0, y = y0;
    while (true) {
      if (inGrid(x, y)) {
        const unsigned int idx = geom_.index(x, y);
        if (stamp_[idx] == kf_id && state_[idx] == kObstacle) break;
        if (stamp_[idx] != kf_id) {
          stamp_[idx] = kf_id;
          state_[idx] = kFree;
          touched.push_back(idx);
        }
      }
      if (x == x1 && y == y1) break;
      const int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 < dx) { err += dx; y += sy; }
    }
  }

  Tier3 classifyReturn(const Eigen::Vector3f& pt,
                       const Eigen::Vector3f* normal) const {
    const double gz = ground_.at(pt.x(), pt.y());
    const int off =
        layerOf(pt.z(), p_.resolution) - layerOf(gz, p_.resolution);
    // Band TOP = the lidar plane (this keyframe's origin z), capped by the fixed
    // safety bound, so the whole sub-lidar volume is obstacle and overhead is not.
    const int top = bandTopLayers(lidar_z_, gz, p_.phantom_drop,
                                  p_.obstacle_max_layers, p_.resolution);
    if (!p_.use_normals || normal == nullptr) {
      return classify3(off, p_.walkable_layers, top);
    }
    bool has_n = false;
    const double ang =
        normalAngleFromUp(normal->x(), normal->y(), normal->z(), has_n);
    return classify3WithNormal(off, p_.walkable_layers, top, has_n, ang,
                               p_.ground_normal_angle, p_.vertical_normal_angle,
                               p_.normal_flat_grace_layers);
  }

  const CostmapGrid& geom_;
  const StaticLayerParams& p_;
  const GroundLookup& ground_;
  OccupancyAccumulator& acc_;
  int nx_, ny_;
  std::vector<int> stamp_;          // last keyframe id that touched a cell
  std::vector<std::uint8_t> state_; // local per-cell state for the current kf
  int nbins_ = 0;
  double step_ = 0.0;
  double lidar_z_ = 0.0;            // current keyframe's lidar plane z (band top)
};

}  // namespace

void erodeObstacleSpeckle(CostmapGrid& g) {
  const int nx = static_cast<int>(g.sizeX());
  const int ny = static_cast<int>(g.sizeY());
  const int dxs[4] = {1, -1, 0, 0};
  const int dys[4] = {0, 0, 1, -1};
  std::vector<unsigned int> to_free;
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      if (g.at(x, y) != LETHAL_OBSTACLE) continue;
      int free_n = 0;
      bool unknown_n = false;
      for (int k = 0; k < 4; ++k) {
        const int xx = x + dxs[k], yy = y + dys[k];
        if (xx < 0 || yy < 0 || xx >= nx || yy >= ny) continue;
        const std::uint8_t c = g.at(xx, yy);
        if (c == FREE_SPACE) ++free_n;
        else if (c == NO_INFORMATION) unknown_n = true;
      }
      if (free_n >= 3 && !unknown_n) to_free.push_back(g.index(x, y));
    }
  }
  for (const unsigned int idx : to_free) g.data()[idx] = FREE_SPACE;
}

bool StaticLayer::build(const std::vector<Eigen::Vector3f>& points_map,
                        const GroundLookup& ground, const StaticLayerParams& p,
                        CostmapGrid& out) const {
  if (points_map.empty() || p.resolution <= 0.0) return false;
  std::vector<float> xs, ys;
  xs.reserve(points_map.size());
  ys.reserve(points_map.size());
  for (const auto& pt : points_map) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  double min_x, min_y, max_x, max_y;
  robustBounds(xs, ys, p.xy_clip_pct, min_x, min_y, max_x, max_y);
  sizeGrid(min_x, min_y, max_x, max_y, p.resolution, p.margin, out);

  unsigned int mx = 0, my = 0;
  for (const auto& pt : points_map) {
    if (!out.worldToMap(pt.x(), pt.y(), mx, my)) continue;
    if (!ground.valid(pt.x(), pt.y())) continue;
    markPoint(pt, ground, p, out, mx, my);
  }
  morphClose(out, p.hole_fill_iters);
  return true;
}

bool StaticLayer::buildFromKeyframes(const std::vector<StaticKeyframe>& kfs,
                                     const GroundLookup& ground,
                                     const StaticLayerParams& p, CostmapGrid& out,
                                     std::vector<std::int8_t>* prob_out) const {
  if (p.resolution <= 0.0) return false;
  std::vector<float> xs, ys;
  for (const auto& kf : kfs) {
    for (const auto& pt : kf.points) {
      xs.push_back(pt.x());
      ys.push_back(pt.y());
    }
  }
  if (xs.empty()) return false;
  double min_x, min_y, max_x, max_y;
  robustBounds(xs, ys, p.xy_clip_pct, min_x, min_y, max_x, max_y);
  sizeGrid(min_x, min_y, max_x, max_y, p.resolution, p.margin, out);

  // Accumulate each keyframe's local grid into a probabilistic (log-odds) map.
  OccupancyAccumulator acc;
  acc.configure(out.cells(), p.occ);
  LocalGridMaker maker(out, p, ground, acc);
  int kf_id = 0;
  for (const auto& kf : kfs) maker.integrate(kf, kf_id++);

  // Threshold to 3-state (cells never observed stay UNKNOWN).
  std::vector<std::uint8_t>& cells = out.data();
  for (std::size_t i = 0; i < cells.size(); ++i) {
    const std::uint8_t s = acc.stateAt(i);
    if (s != NO_INFORMATION) cells[i] = s;
  }

  // Raw fusion confidence (before the cosmetic erode/close), for the prob grid.
  if (prob_out) {
    prob_out->resize(out.cells());
    for (std::size_t i = 0; i < out.cells(); ++i) {
      (*prob_out)[i] = acc.occupancyAt(i);
    }
  }

  if (p.erode_obstacles) erodeObstacleSpeckle(out);
  morphClose(out, p.hole_fill_iters);
  return true;
}

}  // namespace g1_costmap
