// LayeredCostmap unit tests: ground-relative static+dynamic fusion, 3-state
// (free/obstacle/unknown), inflation, occupancy mapping.

#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "g1_costmap/core/cost_values.h"
#include "g1_costmap/layers/layered_costmap.h"

using namespace g1_costmap;

namespace {
CostmapConfig config() {
  CostmapConfig c;
  c.resolution = 0.1;
  c.margin = 0.3;
  c.walkable_layers = 1;
  c.obstacle_max_height = 1.2;
  c.max_fill_radius = 0.3;
  c.ground_min_points = 1;
  c.dyn_max_range = 10.0;
  c.dyn_prob_hit = 0.7;
  c.dyn_prob_miss = 0.4;
  c.dyn_clamp_min = 0.12;
  c.dyn_clamp_max = 0.97;
  c.dyn_decay_half_life = 3.0;
  c.dyn_add_thr = 0.65;
  c.dyn_fill_thr = 0.65;
  c.dyn_clear_thr = 0.85;
  c.dyn_raycast = true;
  c.dyn_obstacle_frac = 0.0;  // fusion tests use single-point obstacles; keep the
  c.dyn_min_returns = 1;      // dynamic layer's legacy "any in-band marks" behavior
  c.robot_radius = 0.2;
  c.inflation_radius = 0.4;
  c.cost_scaling_factor = 3.0;
  return c;
}

std::vector<Eigen::Vector3f> floorWithWall() {
  std::vector<Eigen::Vector3f> pts;
  for (int xi = 0; xi < 10; ++xi) {
    for (int yi = 0; yi < 10; ++yi) {
      const float x = 0.05f + 0.1f * xi;
      const float y = 0.05f + 0.1f * yi;
      pts.emplace_back(x, y, 0.0f);
      pts.emplace_back(x, y, 0.0f);
      pts.emplace_back(x, y, 0.0f);
    }
  }
  pts.emplace_back(0.52f, 0.52f, 0.5f);  // wall column
  return pts;
}
}  // namespace

TEST(LayeredCostmap, FuseWithoutStaticIsEmpty) {
  LayeredCostmap lc(config());
  EXPECT_FALSE(lc.hasStatic());
  EXPECT_FALSE(lc.fuse().initialized());
}

TEST(LayeredCostmap, OccupancyIsThreeStateMasterIsInflated) {
  LayeredCostmap lc(config());
  lc.setFootGround(0.0);
  ASSERT_TRUE(lc.setStaticCloud(floorWithWall()));
  const CostmapGrid& m = lc.fuse();
  const CostmapGrid& occ = lc.occupancy();
  ASSERT_EQ(occ.cells(), m.cells());

  unsigned int wx = 0, wy = 0;
  ASSERT_TRUE(occ.worldToMap(0.52, 0.52, wx, wy));
  EXPECT_EQ(occ.at(wx, wy), LETHAL_OBSTACLE);  // the wall is lethal in both

  // occupancy() is the pre-inflation 3-state grid map: every cell is FREE / LETHAL / UNKNOWN,
  // never an inflation cost; master() carries the inflation band around obstacles.
  bool master_has_inflation = false;
  for (std::size_t i = 0; i < occ.cells(); ++i) {
    const std::uint8_t c = occ.data()[i];
    EXPECT_TRUE(c == FREE_SPACE || c == LETHAL_OBSTACLE || c == NO_INFORMATION);
    const std::uint8_t mc = m.data()[i];
    if (mc != FREE_SPACE && mc != LETHAL_OBSTACLE && mc != NO_INFORMATION) {
      master_has_inflation = true;
    }
  }
  EXPECT_TRUE(master_has_inflation);
}

TEST(LayeredCostmap, UnobservedStaysUnknownAndMapsToMinusOne) {
  LayeredCostmap lc(config());
  lc.setFootGround(0.0);
  ASSERT_TRUE(lc.setStaticCloud(floorWithWall()));
  const CostmapGrid& m = lc.fuse();

  // The grid's far margin corner is never observed and far from any obstacle.
  unsigned int cx = 0, cy = 0;
  ASSERT_TRUE(m.worldToMap(m.originX() + 0.01, m.originY() + 0.01, cx, cy));
  EXPECT_EQ(m.at(cx, cy), NO_INFORMATION);            // inflation never raised it
  EXPECT_EQ(costToOccupancy(m.at(cx, cy)), -1);       // unknown -> -1
}

TEST(LayeredCostmap, KeyframeRaycastClearsFreeBetweenOriginAndWall) {
  LayeredCostmap lc(config());
  lc.setFootGround(0.0);

  // One keyframe: origin near (0,0), a floor swept out to a wall return at 2 m.
  StaticKeyframe kf;
  kf.origin = Eigen::Vector3f(0.05f, 0.05f, 0.0f);
  for (float x = 0.0f; x < 2.0f; x += 0.05f) {
    kf.points.emplace_back(x, 0.05f, 0.0f);  // floor returns (ground)
  }
  kf.points.emplace_back(2.0f, 0.05f, 0.5f);  // wall return (above ground)
  std::vector<StaticKeyframe> kfs{kf};
  ASSERT_TRUE(lc.setStaticKeyframes(kfs));

  const CostmapGrid& m = lc.fuse();
  ASSERT_TRUE(m.initialized());

  unsigned int fx = 0, fy = 0;
  ASSERT_TRUE(m.worldToMap(1.0, 0.05, fx, fy));  // mid-beam, far from the wall
  EXPECT_EQ(m.at(fx, fy), FREE_SPACE);           // raycast-cleared free

  unsigned int wx = 0, wy = 0;
  ASSERT_TRUE(m.worldToMap(2.0, 0.05, wx, wy));
  EXPECT_EQ(m.at(wx, wy), LETHAL_OBSTACLE);  // wall return
}

namespace {
// One keyframe: a swept floor along y=0.05 out to a wall at x=1.5 (LETHAL). The
// raycast stops at the wall, so cells just BEYOND it stay UNKNOWN (occluded).
std::vector<StaticKeyframe> corridorWithWall() {
  StaticKeyframe kf;
  kf.origin = Eigen::Vector3f(0.05f, 0.05f, 0.0f);
  for (float x = 0.0f; x < 1.5f; x += 0.05f) {
    kf.points.emplace_back(x, 0.05f, 0.0f);   // floor returns (ground)
  }
  kf.points.emplace_back(1.5f, 0.05f, 0.5f);  // wall return (above ground)
  return std::vector<StaticKeyframe>{kf};
}
const Eigen::Vector3f kDynSensor(0.05f, 0.05f, 1.0f);  // lidar plane above the band
}  // namespace

TEST(LayeredCostmap, LiveObstacleIsAddedOverStaticFree) {
  LayeredCostmap lc(config());
  lc.setFootGround(0.0);
  ASSERT_TRUE(lc.setStaticCloud(floorWithWall()));

  std::vector<Eigen::Vector3f> live;
  live.emplace_back(0.2f, 0.2f, 0.5f);  // obstacle above ground, below the lidar
  lc.updateDynamic(live, Eigen::Vector3f(0.05f, 0.05f, 1.0f), 1.0);

  const CostmapGrid& m = lc.fuse();
  unsigned int dx = 0, dy = 0;
  ASSERT_TRUE(m.worldToMap(0.2, 0.2, dx, dy));
  EXPECT_EQ(m.at(dx, dy), LETHAL_OBSTACLE);  // one hit crosses dyn_add_thr
}

// A door mapped CLOSED (static LETHAL) is cleared only after SUSTAINED free
// evidence; a static UNKNOWN cell on the same beam fills FREE almost immediately.
// This is the asymmetry: low bar to fill/add, HIGH bar to clear a wall.
TEST(LayeredCostmap, AsymmetricClearVsFill) {
  CostmapConfig cfg = config();
  std::vector<Eigen::Vector3f> open;
  open.emplace_back(1.7f, 0.05f, 0.0f);  // ground return past the wall (x=1.5)

  // After only 2 updates: the unknown cell behind the wall fills FREE, the wall does
  // NOT clear (high bar).
  {
    LayeredCostmap lc(cfg);
    lc.setFootGround(0.0);
    ASSERT_TRUE(lc.setStaticKeyframes(corridorWithWall()));
    for (int i = 0; i < 2; ++i) lc.updateDynamic(open, kDynSensor, 1.0);
    lc.fuse();
    // Assert on the pre-inflation 3-state grid: the still-LETHAL wall inflates its
    // immediate neighbours in master(), which would mask the freshly-filled cell.
    const CostmapGrid& occ = lc.occupancy();
    unsigned int wx = 0, wy = 0;
    ASSERT_TRUE(occ.worldToMap(1.5, 0.05, wx, wy));
    EXPECT_EQ(occ.at(wx, wy), LETHAL_OBSTACLE) << "wall must NOT clear on 2 misses";
    unsigned int ux = 0, uy = 0;
    ASSERT_TRUE(occ.worldToMap(1.6, 0.05, ux, uy));
    EXPECT_EQ(occ.at(ux, uy), FREE_SPACE) << "unknown-behind-wall fills FREE quickly";
  }

  // After sustained updates: the wall (door) clears to FREE.
  {
    LayeredCostmap lc(cfg);
    lc.setFootGround(0.0);
    ASSERT_TRUE(lc.setStaticKeyframes(corridorWithWall()));
    for (int i = 0; i < 14; ++i) lc.updateDynamic(open, kDynSensor, 1.0);
    const CostmapGrid& m = lc.fuse();
    unsigned int wx = 0, wy = 0;
    ASSERT_TRUE(m.worldToMap(1.5, 0.05, wx, wy));
    EXPECT_EQ(m.at(wx, wy), FREE_SPACE) << "sustained free evidence clears the wall";
  }
}

// A pedestrian ghost: a live obstacle appears, then is never re-observed (occluded)
// and decays back to static.
TEST(LayeredCostmap, GhostObstacleDecaysBackToStatic) {
  LayeredCostmap lc(config());
  lc.setFootGround(0.0);
  ASSERT_TRUE(lc.setStaticCloud(floorWithWall()));

  std::vector<Eigen::Vector3f> live;
  live.emplace_back(0.2f, 0.2f, 0.5f);  // dynamic-only obstacle on static FREE
  lc.updateDynamic(live, Eigen::Vector3f(0.05f, 0.05f, 1.0f), 1.0);
  {
    const CostmapGrid& m = lc.fuse();
    unsigned int dx = 0, dy = 0;
    ASSERT_TRUE(m.worldToMap(0.2, 0.2, dx, dy));
    ASSERT_EQ(m.at(dx, dy), LETHAL_OBSTACLE);
  }

  std::vector<Eigen::Vector3f> empty;
  for (int i = 1; i <= 8; ++i) {
    lc.updateDynamic(empty, Eigen::Vector3f(0.05f, 0.05f, 1.0f), 1.0 + 3.0 * i);
  }
  const CostmapGrid& m = lc.fuse();
  unsigned int dx = 0, dy = 0;
  ASSERT_TRUE(m.worldToMap(0.2, 0.2, dx, dy));
  EXPECT_NE(m.at(dx, dy), LETHAL_OBSTACLE) << "ghost must fade back to static FREE";
}

// clearDynamic (clear_costmap service) wipes the change evidence: a cleared door
// reverts to LETHAL until re-observed.
TEST(LayeredCostmap, ClearDynamicResetsChangeEvidence) {
  LayeredCostmap lc(config());
  lc.setFootGround(0.0);
  ASSERT_TRUE(lc.setStaticKeyframes(corridorWithWall()));

  std::vector<Eigen::Vector3f> open;
  open.emplace_back(1.7f, 0.05f, 0.0f);
  for (int i = 0; i < 14; ++i) lc.updateDynamic(open, kDynSensor, 1.0);
  {
    const CostmapGrid& m = lc.fuse();
    unsigned int wx = 0, wy = 0;
    ASSERT_TRUE(m.worldToMap(1.5, 0.05, wx, wy));
    ASSERT_EQ(m.at(wx, wy), FREE_SPACE) << "wall cleared before clearDynamic";
  }

  lc.clearDynamic();
  const CostmapGrid& m2 = lc.fuse();
  unsigned int wx = 0, wy = 0;
  ASSERT_TRUE(m2.worldToMap(1.5, 0.05, wx, wy));
  EXPECT_EQ(m2.at(wx, wy), LETHAL_OBSTACLE) << "wall returns after clearDynamic";
}

// Production in-band-fraction rule (dyn_obstacle_frac/dyn_min_returns ON), through
// the full fuse(): a supported obstacle (most returns in-band, enough returns) is
// added LETHAL; flat-ground scatter (a few in-band returns among many floor
// returns) stays FREE, so no false ring. Asserts on the pre-inflation occupancy so
// the static wall's inflation halo cannot mask the scatter cell.
TEST(LayeredCostmap, FractionRuleAddsSupportedObstacleNotScatter) {
  CostmapConfig cfg = config();
  cfg.dyn_obstacle_frac = 0.4;  // enable the support test (config() pins it off)
  cfg.dyn_min_returns = 3;
  LayeredCostmap lc(cfg);
  lc.setFootGround(0.0);
  ASSERT_TRUE(lc.setStaticCloud(floorWithWall()));

  std::vector<Eigen::Vector3f> live;
  for (int i = 0; i < 4; ++i)
    live.emplace_back(0.25f, 0.25f, 0.5f);  // supported obstacle: 4 in-band, frac 1.0
  for (int i = 0; i < 5; ++i)
    live.emplace_back(0.65f, 0.65f, 0.0f);  // scatter cell: 5 floor returns ...
  live.emplace_back(0.65f, 0.65f, 0.3f);    // ... + 1 in-band -> frac 1/6 < 0.4
  lc.updateDynamic(live, Eigen::Vector3f(0.05f, 0.05f, 1.0f), 1.0);

  lc.fuse();
  const CostmapGrid& occ = lc.occupancy();
  unsigned int ox = 0, oy = 0;
  ASSERT_TRUE(occ.worldToMap(0.25, 0.25, ox, oy));
  EXPECT_EQ(occ.at(ox, oy), LETHAL_OBSTACLE);  // supported -> added over static free

  unsigned int sx = 0, sy = 0;
  ASSERT_TRUE(occ.worldToMap(0.65, 0.65, sx, sy));
  EXPECT_EQ(occ.at(sx, sy), FREE_SPACE);  // scatter -> stays free, no false obstacle
}

TEST(CostToOccupancy, MapsCostBands) {
  EXPECT_EQ(costToOccupancy(FREE_SPACE), 0);
  EXPECT_EQ(costToOccupancy(LETHAL_OBSTACLE), 100);   // real obstacle
  EXPECT_EQ(costToOccupancy(INSCRIBED_INFLATED), 99); // inscribed, distinct from lethal
  EXPECT_EQ(costToOccupancy(NO_INFORMATION), -1);
  const int mid = costToOccupancy(127);
  EXPECT_GT(mid, 0);
  EXPECT_LT(mid, 99);  // inflation band stays below inscribed
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
