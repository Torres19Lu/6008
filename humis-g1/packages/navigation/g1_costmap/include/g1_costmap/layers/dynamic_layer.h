#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "g1_costmap/core/costmap_grid.h"
#include "g1_costmap/core/ground_surface.h"

namespace g1_costmap {

struct DynamicLayerParams {
  double resolution = 0.1;
  int walkable_layers = 1;        // ground-relative band (same as the static layer)
  int obstacle_max_layers = 20;   // safety cap; the lidar plane normally bites first
  double phantom_drop = 0.1;      // m below the lidar plane; above it -> ignored
  double lidar_z = 0.0;           // lidar plane z (band top); used when have_lidar_z
  bool have_lidar_z = false;      // false -> fall back to obstacle_max_layers only
  double max_range = 15.0;        // m; ignore returns beyond this horizontal range
  bool raycast = true;            // vote a miss on cells a beam passes through
  // Support test (RTAB-Map cluster-min-size analog): a cell is OBSTACLE only when a
  // sufficient FRACTION of its returns are in-band, and it has enough returns to
  // trust that fraction. NOTE the asymmetry: these PRIMITIVE defaults (0.0 / 1)
  // reproduce the legacy "any in-band return marks the cell" behaviour, and DIFFER
  // from the production defaults the owner injects (CostmapConfig dyn_obstacle_frac
  // = 0.4, dyn_min_returns = 3). Construct DynamicLayerParams directly and you get
  // the no-op unless you set these; the node path is the configured one.
  double obstacle_frac = 0.0;     // mark OBSTACLE iff n_band/n_total > this
  int min_returns = 1;            // need this many returns in the cell to apply it
  // Bidirectional log-odds evidence (replaces the old binary decay_time forget).
  double prob_hit = 0.7;          // P(occupied | obstacle return); > 0.5
  double prob_miss = 0.4;         // P(occupied | free/miss observation); < 0.5
  double clamp_min = 0.12;        // lower probability clamp (free saturation)
  double clamp_max = 0.97;        // upper probability clamp (obstacle saturation)
  double decay_half_life = 3.0;   // s; evidence relaxes toward neutral by half each
};

// Signed per-cell CHANGE evidence (log-odds) vs the static SLAM map, from the live
// registered cloud, classified against the same per-cell ground (GroundLookup) and
// ground->lidar-plane band as the static layer. A raycast votes a MISS (lowers the
// log-odds) on cells a beam passes through (stopping at this update's obstacles, the
// occlusion guard); an obstacle return votes a HIT (raises it). Every tick all
// non-neutral cells relax toward 0 by a half-life, so an unobserved obstacle ghost
// fades while a re-observed cell holds. The fusion layer thresholds this against the
// static map (asymmetric: add/fill low bar, clear-static HIGH bar). The SLAM map is
// never modified.
class DynamicLayer {
 public:
  void configure(const CostmapGrid& geometry);  // adopt geometry, reset state
  // Reset all change evidence to neutral (0) and the decay clock, so the next
  // fuse() defers fully to the static map. Geometry is preserved. Called by the
  // clear_costmap service (D7b-6) via LayeredCostmap::clearDynamic().
  void clear();
  bool initialized() const { return geom_.initialized(); }

  void update(const std::vector<Eigen::Vector3f>& points_map,
              const Eigen::Vector3f& sensor_origin, const GroundLookup& ground,
              const DynamicLayerParams& p, double now);

  // Signed change evidence at a cell: > 0 leans occupied, < 0 leans free, 0 neutral.
  float logoddsAt(unsigned int mx, unsigned int my) const {
    return logodds_[geom_.index(mx, my)];
  }
  const std::vector<float>& logodds() const { return logodds_; }
  const CostmapGrid& geometry() const { return geom_; }

 private:
  // Bresenham from (x0,y0) toward (x1,y1); vote a MISS (l_miss, clamped at l_min) on
  // every traversed cell except the endpoint, at most once per cell per update, and
  // STOP at the first cell flagged in obs_mask_ (an obstacle observed this update).
  void raytraceMissStop(int x0, int y0, int x1, int y1, float l_miss, float l_min);

  CostmapGrid geom_;                    // geometry only; its cost grid is unused
  std::vector<float> logodds_;          // signed change evidence per cell
  std::vector<std::uint8_t> obs_mask_;  // obstacle cells of the current update (stop)
  std::vector<std::uint8_t> voted_;     // cells already voted this update (1 vote/cell)
  std::vector<std::uint16_t> cell_total_;  // per-cell non-IGNORED return count (this update)
  std::vector<std::uint16_t> cell_band_;   // per-cell in-band return count (this update)
  std::vector<std::size_t> touched_;    // indices voted this update (to reset voted_)
  double prev_time_ = 0.0;              // last update time (decay clock)
  bool has_prev_ = false;
};

}  // namespace g1_costmap
