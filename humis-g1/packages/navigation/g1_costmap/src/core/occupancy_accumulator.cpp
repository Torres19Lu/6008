#include "g1_costmap/core/occupancy_accumulator.h"

#include <algorithm>
#include <cmath>

namespace g1_costmap {

void OccupancyAccumulator::configure(std::size_t cells,
                                     const OccupancyAccumulatorParams& p) {
  params_ = p;
  l_hit_ = static_cast<float>(logOdds(p.prob_hit));
  l_miss_ = static_cast<float>(logOdds(p.prob_miss));
  l_min_ = static_cast<float>(logOdds(p.prob_clamp_min));
  l_max_ = static_cast<float>(logOdds(p.prob_clamp_max));
  l_thr_ = static_cast<float>(logOdds(p.occupancy_thr));
  logodds_.assign(cells, 0.0f);
  seen_.assign(cells, 0);
}

void OccupancyAccumulator::reset() {
  std::fill(logodds_.begin(), logodds_.end(), 0.0f);
  std::fill(seen_.begin(), seen_.end(), 0);
}

void OccupancyAccumulator::observeHit(std::size_t idx) {
  float v = logodds_[idx] + l_hit_;
  logodds_[idx] = std::min(l_max_, std::max(l_min_, v));
  seen_[idx] = 1;
}

void OccupancyAccumulator::observeMiss(std::size_t idx) {
  float v = logodds_[idx] + l_miss_;
  logodds_[idx] = std::min(l_max_, std::max(l_min_, v));
  seen_[idx] = 1;
}

double OccupancyAccumulator::probabilityAt(std::size_t idx) const {
  return probFromLogOdds(logodds_[idx]);
}

std::int8_t OccupancyAccumulator::occupancyAt(std::size_t idx) const {
  if (!seen_[idx]) return -1;
  const int occ = static_cast<int>(std::lround(probabilityAt(idx) * 100.0));
  return static_cast<std::int8_t>(std::min(100, std::max(0, occ)));
}

std::uint8_t OccupancyAccumulator::stateAt(std::size_t idx) const {
  if (!seen_[idx]) return NO_INFORMATION;
  return logodds_[idx] >= l_thr_ ? LETHAL_OBSTACLE : FREE_SPACE;
}

}  // namespace g1_costmap
