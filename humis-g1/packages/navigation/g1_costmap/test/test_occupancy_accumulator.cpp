// OccupancyAccumulator unit tests: log-odds update, clamping, 3-state threshold,
// probability output, and the key robustness property (a noise obstacle gets
// voted back out by repeated free observations).

#include <gtest/gtest.h>

#include "g1_costmap/core/cost_values.h"
#include "g1_costmap/core/occupancy_accumulator.h"

using namespace g1_costmap;

namespace {
OccupancyAccumulatorParams params() {
  OccupancyAccumulatorParams p;
  p.prob_hit = 0.7;
  p.prob_miss = 0.4;
  p.prob_clamp_min = 0.12;
  p.prob_clamp_max = 0.97;
  p.occupancy_thr = 0.5;
  return p;
}
}  // namespace

TEST(OccupancyAccumulator, UnseenIsUnknown) {
  OccupancyAccumulator acc;
  acc.configure(4, params());
  EXPECT_FALSE(acc.observed(0));
  EXPECT_EQ(acc.stateAt(0), NO_INFORMATION);
  EXPECT_EQ(acc.occupancyAt(0), -1);
}

TEST(OccupancyAccumulator, SingleHitIsObstacleSingleMissIsFree) {
  OccupancyAccumulator acc;
  acc.configure(4, params());
  acc.observeHit(0);
  acc.observeMiss(1);
  EXPECT_EQ(acc.stateAt(0), LETHAL_OBSTACLE);
  EXPECT_EQ(acc.stateAt(1), FREE_SPACE);
  EXPECT_GT(acc.probabilityAt(0), 0.5);
  EXPECT_LT(acc.probabilityAt(1), 0.5);
}

TEST(OccupancyAccumulator, RepeatedFreeVotesOutANoiseObstacle) {
  // One spurious obstacle hit, then many free observations from other viewpoints:
  // the cell must end up FREE (the property last-write-wins cannot provide).
  OccupancyAccumulator acc;
  acc.configure(1, params());
  acc.observeHit(0);
  EXPECT_EQ(acc.stateAt(0), LETHAL_OBSTACLE);
  for (int i = 0; i < 10; ++i) acc.observeMiss(0);
  EXPECT_EQ(acc.stateAt(0), FREE_SPACE);
}

TEST(OccupancyAccumulator, StrongFreeResistsASingleNoiseHit) {
  // A heavily-swept free cell (many misses, clamped) is not flipped to obstacle
  // by one spurious hit -> free space is stable against isolated noise.
  OccupancyAccumulator acc;
  acc.configure(1, params());
  for (int i = 0; i < 10; ++i) acc.observeMiss(0);
  acc.observeHit(0);
  EXPECT_EQ(acc.stateAt(0), FREE_SPACE);
}

TEST(OccupancyAccumulator, ClampsSaturation) {
  OccupancyAccumulator acc;
  acc.configure(1, params());
  for (int i = 0; i < 1000; ++i) acc.observeHit(0);
  EXPECT_LE(acc.probabilityAt(0), 0.97 + 1e-6);
  for (int i = 0; i < 5000; ++i) acc.observeMiss(0);
  EXPECT_GE(acc.probabilityAt(0), 0.12 - 1e-6);
}

TEST(OccupancyAccumulator, OccupancyMapsToRange) {
  OccupancyAccumulator acc;
  acc.configure(2, params());
  acc.observeHit(0);
  acc.observeMiss(1);
  const int occ0 = acc.occupancyAt(0);
  const int occ1 = acc.occupancyAt(1);
  EXPECT_GT(occ0, 50);
  EXPECT_LE(occ0, 100);
  EXPECT_LT(occ1, 50);
  EXPECT_GE(occ1, 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
