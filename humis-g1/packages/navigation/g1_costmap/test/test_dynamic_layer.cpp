// DynamicLayer unit tests: signed log-odds change evidence, raycast miss voting,
// obstacle hit voting, exponential decay toward neutral, band + range gating.

#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "g1_costmap/core/costmap_grid.h"
#include "g1_costmap/core/ground_surface.h"
#include "g1_costmap/layers/dynamic_layer.h"

using namespace g1_costmap;

namespace {
DynamicLayerParams params() {
  DynamicLayerParams p;
  p.resolution = 0.1;
  p.walkable_layers = 1;
  p.obstacle_max_layers = 12;
  p.max_range = 10.0;
  p.raycast = true;
  p.prob_hit = 0.7;
  p.prob_miss = 0.4;
  p.clamp_min = 0.12;
  p.clamp_max = 0.97;
  p.decay_half_life = 3.0;
  return p;
}
const Eigen::Vector3f kSensor(0.25f, 0.25f, 0.0f);  // cell (2,2) at res 0.1
const GroundLookup kGround{nullptr, 0.0, true};      // foot scalar ground = 0
}  // namespace

TEST(DynamicLayer, ObstacleRaisesLogoddsAndBeamLowersIt) {
  CostmapGrid geom(0.1, 0.0, 0.0, 50, 50);
  DynamicLayer dyn;
  dyn.configure(geom);

  std::vector<Eigen::Vector3f> pts;
  pts.emplace_back(2.0f, 0.25f, 0.5f);  // 5 layers above ground -> obstacle
  dyn.update(pts, kSensor, kGround, params(), 1.0);

  unsigned int hx = 0, hy = 0;
  ASSERT_TRUE(geom.worldToMap(2.0, 0.25, hx, hy));
  EXPECT_GT(dyn.logoddsAt(hx, hy), 0.0f);  // hit -> leans occupied

  unsigned int mx = 0, my = 0;
  ASSERT_TRUE(geom.worldToMap(1.0, 0.25, mx, my));  // on the beam path
  EXPECT_LT(dyn.logoddsAt(mx, my), 0.0f);  // miss -> leans free
}

TEST(DynamicLayer, ReturnAtGroundLowersLogodds) {
  CostmapGrid geom(0.1, 0.0, 0.0, 50, 50);
  DynamicLayer dyn;
  dyn.configure(geom);

  std::vector<Eigen::Vector3f> pts;
  pts.emplace_back(1.0f, 0.25f, 0.0f);  // at ground -> free endpoint
  dyn.update(pts, kSensor, kGround, params(), 1.0);
  unsigned int hx = 0, hy = 0;
  ASSERT_TRUE(geom.worldToMap(1.0, 0.25, hx, hy));
  EXPECT_LT(dyn.logoddsAt(hx, hy), 0.0f);
}

TEST(DynamicLayer, EvidenceDecaysTowardNeutral) {
  CostmapGrid geom(0.1, 0.0, 0.0, 50, 50);
  DynamicLayer dyn;
  dyn.configure(geom);
  const DynamicLayerParams p = params();

  std::vector<Eigen::Vector3f> pts;
  pts.emplace_back(2.0f, 0.25f, 0.5f);
  dyn.update(pts, kSensor, kGround, p, 1.0);
  unsigned int hx = 0, hy = 0;
  ASSERT_TRUE(geom.worldToMap(2.0, 0.25, hx, hy));
  const float l1 = dyn.logoddsAt(hx, hy);
  ASSERT_GT(l1, 0.0f);

  // Many half-lives later with no observation: relaxes toward 0 but never flips.
  std::vector<Eigen::Vector3f> empty;
  dyn.update(empty, kSensor, kGround, p, 1.0 + 10.0 * p.decay_half_life);
  const float l2 = dyn.logoddsAt(hx, hy);
  EXPECT_GT(l2, 0.0f);
  EXPECT_LT(l2, l1);
}

TEST(DynamicLayer, IgnoresAboveBandAndOutOfRangeStayNeutral) {
  CostmapGrid geom(0.1, 0.0, 0.0, 60, 60);
  DynamicLayer dyn;
  dyn.configure(geom);
  DynamicLayerParams p = params();

  std::vector<Eigen::Vector3f> high;
  high.emplace_back(2.0f, 0.25f, 2.0f);  // 20 layers > obstacle_max -> ignored
  dyn.update(high, kSensor, kGround, p, 1.0);
  unsigned int bx = 0, by = 0;
  ASSERT_TRUE(geom.worldToMap(2.0, 0.25, bx, by));
  EXPECT_FLOAT_EQ(dyn.logoddsAt(bx, by), 0.0f);

  p.max_range = 1.0;
  std::vector<Eigen::Vector3f> far_pts;
  far_pts.emplace_back(3.0f, 0.25f, 0.5f);  // ~2.75 m > max_range
  dyn.update(far_pts, kSensor, kGround, p, 2.0);
  unsigned int fx = 0, fy = 0;
  ASSERT_TRUE(geom.worldToMap(3.0, 0.25, fx, fy));
  EXPECT_FLOAT_EQ(dyn.logoddsAt(fx, fy), 0.0f);
}

TEST(DynamicLayer, ClearResetsEvidenceToNeutral) {
  CostmapGrid geom(0.1, 0.0, 0.0, 50, 50);
  DynamicLayer dyn;
  dyn.configure(geom);
  std::vector<Eigen::Vector3f> pts;
  pts.emplace_back(2.0f, 0.25f, 0.5f);
  dyn.update(pts, kSensor, kGround, params(), 1.0);
  unsigned int hx = 0, hy = 0;
  ASSERT_TRUE(geom.worldToMap(2.0, 0.25, hx, hy));
  ASSERT_NE(dyn.logoddsAt(hx, hy), 0.0f);

  dyn.clear();
  EXPECT_FLOAT_EQ(dyn.logoddsAt(hx, hy), 0.0f);
}

// A flat-ground cell: many floor returns plus a few in-band scatter returns. The
// in-band fraction is below the threshold, so the cell is NOT marked obstacle and
// votes free -- no false ring.
TEST(DynamicLayer, ScatterCellBelowFractionVotesFree) {
  CostmapGrid geom(0.1, 0.0, 0.0, 50, 50);
  DynamicLayer dyn;
  dyn.configure(geom);
  DynamicLayerParams p = params();
  p.obstacle_frac = 0.4;
  p.min_returns = 3;

  std::vector<Eigen::Vector3f> pts;
  for (int i = 0; i < 4; ++i) pts.emplace_back(2.0f, 0.25f, 0.0f);  // floor (offset 0)
  pts.emplace_back(2.0f, 0.25f, 0.3f);                              // scatter (offset 3)
  dyn.update(pts, kSensor, kGround, p, 1.0);  // frac = 1/5 = 0.2 < 0.4

  unsigned int hx = 0, hy = 0;
  ASSERT_TRUE(geom.worldToMap(2.0, 0.25, hx, hy));
  EXPECT_LT(dyn.logoddsAt(hx, hy), 0.0f);  // unsupported -> votes free
}

// A real obstacle cell: most returns are in-band, with enough returns. Marked
// OBSTACLE (votes a hit), so the gate does not blind the layer to obstacles.
TEST(DynamicLayer, SupportedObstacleCellVotesHit) {
  CostmapGrid geom(0.1, 0.0, 0.0, 50, 50);
  DynamicLayer dyn;
  dyn.configure(geom);
  DynamicLayerParams p = params();
  p.obstacle_frac = 0.4;
  p.min_returns = 3;

  std::vector<Eigen::Vector3f> pts;
  for (int i = 0; i < 4; ++i) pts.emplace_back(2.0f, 0.25f, 0.5f);  // in-band (offset 5)
  dyn.update(pts, kSensor, kGround, p, 1.0);  // frac = 4/4 = 1.0 > 0.4, n >= 3

  unsigned int hx = 0, hy = 0;
  ASSERT_TRUE(geom.worldToMap(2.0, 0.25, hx, hy));
  EXPECT_GT(dyn.logoddsAt(hx, hy), 0.0f);  // supported -> hit
}

// A cell with too few returns (< min_returns), all in-band: left NEUTRAL (not
// marked, not voted free) so the temporal log-odds decides over time.
TEST(DynamicLayer, SparseInBandCellStaysNeutral) {
  CostmapGrid geom(0.1, 0.0, 0.0, 50, 50);
  DynamicLayer dyn;
  dyn.configure(geom);
  DynamicLayerParams p = params();
  p.obstacle_frac = 0.4;
  p.min_returns = 3;

  std::vector<Eigen::Vector3f> pts;
  pts.emplace_back(2.0f, 0.25f, 0.5f);
  pts.emplace_back(2.0f, 0.25f, 0.5f);  // only 2 returns (< 3)
  dyn.update(pts, kSensor, kGround, p, 1.0);

  unsigned int hx = 0, hy = 0;
  ASSERT_TRUE(geom.worldToMap(2.0, 0.25, hx, hy));
  EXPECT_FLOAT_EQ(dyn.logoddsAt(hx, hy), 0.0f);  // deferred to temporal, no mark
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
