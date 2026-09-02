#include "g1_costmap/layers/dynamic_layer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>
#include <vector>

#include "g1_costmap/core/ground_classifier.h"
#include "g1_costmap/core/occupancy_accumulator.h"  // logOdds

namespace g1_costmap {

namespace {
constexpr double kLn2 = 0.6931471805599453;
}  // namespace

void DynamicLayer::configure(const CostmapGrid& geometry) {
  geom_.resize(geometry.resolution(), geometry.originX(), geometry.originY(),
               geometry.sizeX(), geometry.sizeY(), NO_INFORMATION);
  logodds_.assign(geom_.cells(), 0.0f);
  obs_mask_.assign(geom_.cells(), 0);
  voted_.assign(geom_.cells(), 0);
  cell_total_.assign(geom_.cells(), 0);
  cell_band_.assign(geom_.cells(), 0);
  touched_.clear();
  prev_time_ = 0.0;
  has_prev_ = false;
}

void DynamicLayer::clear() {
  logodds_.assign(geom_.cells(), 0.0f);
  obs_mask_.assign(geom_.cells(), 0);
  voted_.assign(geom_.cells(), 0);
  cell_total_.assign(geom_.cells(), 0);
  cell_band_.assign(geom_.cells(), 0);
  touched_.clear();
  prev_time_ = 0.0;
  has_prev_ = false;
}

void DynamicLayer::raytraceMissStop(int x0, int y0, int x1, int y1, float l_miss,
                                    float l_min) {
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  int x = x0, y = y0;
  while (x != x1 || y != y1) {
    if (x >= 0 && y >= 0 && x < static_cast<int>(geom_.sizeX()) &&
        y < static_cast<int>(geom_.sizeY())) {
      const unsigned int idx = geom_.index(x, y);
      if (obs_mask_[idx]) break;  // occluded by a current obstacle
      if (!voted_[idx]) {
        voted_[idx] = 1;
        touched_.push_back(idx);
        logodds_[idx] = std::max(logodds_[idx] + l_miss, l_min);
      }
    }
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

void DynamicLayer::update(const std::vector<Eigen::Vector3f>& points_map,
                          const Eigen::Vector3f& sensor_origin,
                          const GroundLookup& ground,
                          const DynamicLayerParams& p, double now) {
  if (!geom_.initialized()) return;

  // 1. Decay all non-neutral cells toward 0 (exponential, half-life). Visible cells
  //    are re-voted below, so decay only governs occluded cells (ghosts fade).
  if (has_prev_ && p.decay_half_life > 0.0) {
    const double dt = now - prev_time_;
    if (dt > 0.0) {
      const float f = static_cast<float>(std::exp(-dt * kLn2 / p.decay_half_life));
      for (float& l : logodds_) l *= f;
    }
  }
  has_prev_ = true;
  prev_time_ = now;

  const float l_hit = static_cast<float>(logOdds(p.prob_hit));
  const float l_miss = static_cast<float>(logOdds(p.prob_miss));
  const float l_min = static_cast<float>(logOdds(p.clamp_min));
  const float l_max = static_cast<float>(logOdds(p.clamp_max));

  unsigned int sx_u = 0, sy_u = 0;
  const bool sensor_in_grid =
      geom_.worldToMap(sensor_origin.x(), sensor_origin.y(), sx_u, sy_u);

  std::vector<unsigned int> obstacle_hits;
  std::vector<unsigned int> free_hits;
  std::vector<std::pair<int, int>> beams;  // one endpoint per touched cell (raycast)
  std::vector<unsigned int> cells;         // unique touched cells this update
  const double max_range_sq = p.max_range * p.max_range;
  unsigned int mx = 0, my = 0;

  // Pass 1: tally per cell the non-IGNORED return count and the in-band count.
  for (const auto& pt : points_map) {
    const double ddx = pt.x() - sensor_origin.x();
    const double ddy = pt.y() - sensor_origin.y();
    if (ddx * ddx + ddy * ddy > max_range_sq) continue;
    if (!geom_.worldToMap(pt.x(), pt.y(), mx, my)) continue;
    if (!ground.valid(pt.x(), pt.y())) continue;
    const double gz = ground.at(pt.x(), pt.y());
    const int top = p.have_lidar_z
                        ? bandTopLayers(p.lidar_z, gz, p.phantom_drop,
                                        p.obstacle_max_layers, p.resolution)
                        : p.obstacle_max_layers;
    const Tier3 tier = classify3(
        layerOf(pt.z(), p.resolution) - layerOf(gz, p.resolution),
        p.walkable_layers, top);
    if (tier == Tier3::IGNORED) continue;  // above the lidar plane (ceiling/ghost)
    const unsigned int idx = geom_.index(mx, my);
    if (cell_total_[idx] == 0) {           // first return in this cell this update
      cells.push_back(idx);
      beams.emplace_back(static_cast<int>(mx), static_cast<int>(my));
    }
    ++cell_total_[idx];
    if (tier == Tier3::OBSTACLE) ++cell_band_[idx];
  }

  // Decision: OBSTACLE only when the in-band fraction is supported (and there are
  // enough returns to trust it). A sparse cell with in-band returns is left NEUTRAL
  // (no vote, no obs_mask) so the temporal log-odds decides; a sparse all-floor cell
  // is FREE. This is the grid-native RTAB-Map cluster-min-size analog.
  for (const unsigned int idx : cells) {
    const int ntot = cell_total_[idx];
    const int nband = cell_band_[idx];
    if (ntot >= p.min_returns) {
      if (static_cast<double>(nband) > p.obstacle_frac * ntot) {
        obstacle_hits.push_back(idx);
        obs_mask_[idx] = 1;
      } else {
        free_hits.push_back(idx);
      }
    } else if (nband == 0) {
      free_hits.push_back(idx);
    }
  }

  // Pass 2: cast each beam, voting MISS on traversed cells (stop at obstacles), only
  // with a valid in-grid sensor pose (an off-grid origin must not paint free).
  if (sensor_in_grid && p.raycast) {
    for (const auto& b : beams) {
      raytraceMissStop(static_cast<int>(sx_u), static_cast<int>(sy_u), b.first,
                       b.second, l_miss, l_min);
    }
  }

  // Free endpoints: MISS vote (skip cells that are an obstacle this update).
  if (sensor_in_grid) {
    for (const unsigned int idx : free_hits) {
      if (obs_mask_[idx] || voted_[idx]) continue;
      voted_[idx] = 1;
      touched_.push_back(idx);
      logodds_[idx] = std::max(logodds_[idx] + l_miss, l_min);
    }
  }

  // Obstacle endpoints: HIT vote (wins), then clear the per-update obstacle mask.
  for (const unsigned int idx : obstacle_hits) {
    if (!voted_[idx]) {
      voted_[idx] = 1;
      touched_.push_back(idx);
      logodds_[idx] = std::min(logodds_[idx] + l_hit, l_max);
    }
    obs_mask_[idx] = 0;
  }

  // Reset the per-update vote marks (only the touched cells).
  for (const std::size_t idx : touched_) voted_[idx] = 0;
  touched_.clear();
  // Reset the per-cell tallies (only the touched cells).
  for (const unsigned int idx : cells) {
    cell_total_[idx] = 0;
    cell_band_[idx] = 0;
  }
}

}  // namespace g1_costmap
