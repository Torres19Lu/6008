#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "g1_costmap/core/cost_values.h"

namespace g1_costmap {

// Probabilistic occupancy parameters (log-odds convention). A cell
// accumulates log-odds: each obstacle observation adds logodds(prob_hit) (> 0),
// each free observation adds logodds(prob_miss) (< 0), clamped to the
// [prob_clamp_min, prob_clamp_max] band. The cell is OBSTACLE once its
// probability reaches occupancy_thr, otherwise FREE; a cell never observed stays
// unknown.
struct OccupancyAccumulatorParams {
  double prob_hit = 0.7;        // P(occupied | obstacle observation); > 0.5
  double prob_miss = 0.4;       // P(occupied | free observation);     < 0.5
  double prob_clamp_min = 0.12; // lower probability clamp (free saturation)
  double prob_clamp_max = 0.97; // upper probability clamp (obstacle saturation)
  double occupancy_thr = 0.5;   // probability at/above which a cell reads OBSTACLE
};

// p in (0,1) -> log-odds; clamped off the open-interval ends so log/exp stay finite.
inline double logOdds(double p) {
  if (p <= 0.0) p = 1e-6;
  if (p >= 1.0) p = 1.0 - 1e-6;
  return std::log(p / (1.0 - p));
}
inline double probFromLogOdds(double l) { return 1.0 - 1.0 / (1.0 + std::exp(l)); }

// Bayesian (log-odds) occupancy fusion over a fixed-size cell grid. Unlike a
// last-write-wins grid, a cell takes many consistent observations to read as an
// obstacle, and later free observations can vote a noise/drift obstacle back out.
// ROS-free; indexed by the owning CostmapGrid's row-major cell index.
class OccupancyAccumulator {
 public:
  // (Re)size to `cells` cells, all unknown (log-odds 0, unseen).
  void configure(std::size_t cells, const OccupancyAccumulatorParams& p);
  void reset();

  void observeHit(std::size_t idx);   // one obstacle observation at this cell
  void observeMiss(std::size_t idx);  // one free observation at this cell

  bool observed(std::size_t idx) const { return seen_[idx] != 0; }
  double probabilityAt(std::size_t idx) const;  // 0..1 (0.5 if observed but neutral)

  // nav_msgs/OccupancyGrid value: -1 unknown, else round(probability * 100).
  std::int8_t occupancyAt(std::size_t idx) const;
  // 3-state cost: NO_INFORMATION (unseen) / LETHAL_OBSTACLE / FREE_SPACE.
  std::uint8_t stateAt(std::size_t idx) const;

  std::size_t size() const { return logodds_.size(); }

 private:
  OccupancyAccumulatorParams params_;
  float l_hit_ = 0.0f;   // logodds(prob_hit), > 0
  float l_miss_ = 0.0f;  // logodds(prob_miss), < 0
  float l_min_ = 0.0f;   // logodds(prob_clamp_min)
  float l_max_ = 0.0f;   // logodds(prob_clamp_max)
  float l_thr_ = 0.0f;   // logodds(occupancy_thr)
  std::vector<float> logodds_;     // accumulated log-odds per cell
  std::vector<std::uint8_t> seen_; // whether a cell ever took an observation
};

}  // namespace g1_costmap
