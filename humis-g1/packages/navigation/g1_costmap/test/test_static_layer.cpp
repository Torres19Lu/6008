// StaticLayer unit tests: ground-relative 3-state projection (free/obstacle/unknown).

#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "g1_costmap/core/ground_surface.h"
#include "g1_costmap/layers/static_layer.h"

using namespace g1_costmap;

namespace {
int countCost(const CostmapGrid& g, std::uint8_t v) {
  int n = 0;
  for (auto c : g.data()) {
    if (c == v) ++n;
  }
  return n;
}

// A flat floor (z=0) across [0,1)x[0,1) at >= min_points per cell, plus one wall
// column rising to z=0.5 at (0.5,0.5).
std::vector<Eigen::Vector3f> floorWithWall() {
  std::vector<Eigen::Vector3f> pts;
  for (int xi = 0; xi < 10; ++xi) {
    for (int yi = 0; yi < 10; ++yi) {
      const float x = 0.05f + 0.1f * xi;
      const float y = 0.05f + 0.1f * yi;
      pts.emplace_back(x, y, 0.0f);
      pts.emplace_back(x, y, 0.0f);
      pts.emplace_back(x, y, 0.0f);  // 3 -> trusted ground
    }
  }
  pts.emplace_back(0.52f, 0.52f, 0.5f);  // wall: 5 layers above ground
  return pts;
}
}  // namespace

TEST(StaticLayer, GroundRelativeThreeState) {
  const std::vector<Eigen::Vector3f> pts = floorWithWall();
  GroundSurface gs(0.1, 0.3, 3);
  gs.update(pts);
  GroundLookup gl{&gs, 0.0, true};

  StaticLayer layer;
  StaticLayerParams p;
  p.resolution = 0.1;
  p.margin = 0.3;
  p.walkable_layers = 1;
  p.obstacle_max_layers = 12;

  CostmapGrid g;
  ASSERT_TRUE(layer.build(pts, gl, p, g));

  unsigned int fx = 0, fy = 0;
  ASSERT_TRUE(g.worldToMap(0.05, 0.05, fx, fy));
  EXPECT_EQ(g.at(fx, fy), FREE_SPACE);  // floor -> free

  unsigned int wx = 0, wy = 0;
  ASSERT_TRUE(g.worldToMap(0.52, 0.52, wx, wy));
  EXPECT_EQ(g.at(wx, wy), LETHAL_OBSTACLE);  // wall -> obstacle (above the band)

  EXPECT_GT(countCost(g, NO_INFORMATION), 0);  // the margin ring is unobserved
  EXPECT_EQ(countCost(g, LETHAL_OBSTACLE), 1);  // only the wall column
}

TEST(StaticLayer, BelowGroundCountsAsFree) {
  // A point below the ground reference is traversable (owner model).
  std::vector<Eigen::Vector3f> pts;
  pts.emplace_back(1.0f, 1.0f, -0.3f);  // 3 layers below ground 0
  GroundLookup gl{nullptr, 0.0, true};  // foot scalar ground = 0, no surface
  StaticLayer layer;
  StaticLayerParams p;
  p.resolution = 0.1;
  p.margin = 0.2;
  p.walkable_layers = 1;
  p.obstacle_max_layers = 12;
  CostmapGrid g;
  ASSERT_TRUE(layer.build(pts, gl, p, g));
  unsigned int mx = 0, my = 0;
  ASSERT_TRUE(g.worldToMap(1.0, 1.0, mx, my));
  EXPECT_EQ(g.at(mx, my), FREE_SPACE);
}

TEST(StaticLayer, HoleFillClosesEnclosedGap) {
  // A 5x5 floor with one missing cell in the centre; the enclosed gap must be
  // filled FREE by the morphological close (the voxel-sparse "punched dots").
  std::vector<Eigen::Vector3f> pts;
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      if (i == 2 && j == 2) continue;  // the hole
      pts.emplace_back(0.05f + 0.1f * i, 0.05f + 0.1f * j, 0.0f);
    }
  }
  GroundSurface gs(0.1, 0.3, 1);
  gs.update(pts);
  GroundLookup gl{&gs, 0.0, true};
  StaticLayer layer;
  StaticLayerParams p;
  p.resolution = 0.1;
  p.margin = 0.2;
  p.walkable_layers = 1;
  p.obstacle_max_layers = 12;
  p.hole_fill_iters = 1;
  CostmapGrid g;
  ASSERT_TRUE(layer.build(pts, gl, p, g));
  unsigned int hx = 0, hy = 0;
  ASSERT_TRUE(g.worldToMap(0.25, 0.25, hx, hy));  // the centre hole
  EXPECT_EQ(g.at(hx, hy), FREE_SPACE);
}

TEST(StaticLayer, EmptyCloudReturnsFalse) {
  StaticLayer layer;
  StaticLayerParams p;
  GroundLookup gl{nullptr, 0.0, true};
  std::vector<Eigen::Vector3f> pts;
  CostmapGrid g;
  EXPECT_FALSE(layer.build(pts, gl, p, g));
  EXPECT_FALSE(g.initialized());
}

TEST(StaticLayer, ErodeRemovesIsolatedObstacleSpeckle) {
  CostmapGrid g(0.1, 0.0, 0.0, 5, 5, FREE_SPACE);
  g.setCost(2, 2, LETHAL_OBSTACLE);  // a single noise obstacle ringed by free
  erodeObstacleSpeckle(g);
  EXPECT_EQ(g.at(2, 2), FREE_SPACE);
}

TEST(StaticLayer, ErodeKeepsObstacleBorderingUnknown) {
  CostmapGrid g(0.1, 0.0, 0.0, 5, 5, NO_INFORMATION);
  g.setCost(2, 2, LETHAL_OBSTACLE);
  g.setCost(1, 2, FREE_SPACE);
  g.setCost(3, 2, FREE_SPACE);
  g.setCost(2, 1, FREE_SPACE);  // (2,3) stays unknown -> obstacle is the map frontier
  erodeObstacleSpeckle(g);
  EXPECT_EQ(g.at(2, 2), LETHAL_OBSTACLE);  // touches unknown -> not eroded
}

TEST(StaticLayer, KeyframeLogOddsVotesOutNoiseObstacle) {
  // Flat ground (foot scalar 0). One keyframe sees a spurious obstacle at C=(1,0);
  // three later keyframes raycast free through C from the far side. Last-write
  // would leave C an obstacle forever; log-odds votes it back out to FREE.
  GroundLookup gl{nullptr, 0.0, true};
  StaticLayer layer;
  StaticLayerParams p;
  p.resolution = 0.1;
  p.margin = 1.0;
  p.walkable_layers = 1;
  p.obstacle_max_layers = 12;
  p.xy_clip_pct = 0.0;  // tiny synthetic cloud: keep the true XY extent

  StaticKeyframe bad;
  bad.origin = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  bad.points.emplace_back(1.0f, 0.0f, 0.5f);  // spurious obstacle at C

  auto good = []() {
    StaticKeyframe k;
    k.origin = Eigen::Vector3f(2.0f, 0.0f, 0.0f);
    k.points.emplace_back(0.0f, 0.0f, 0.0f);  // floor return; the ray crosses C
    return k;
  };

  {
    CostmapGrid g;
    ASSERT_TRUE(layer.buildFromKeyframes({bad}, gl, p, g));
    unsigned int cx = 0, cy = 0;
    ASSERT_TRUE(g.worldToMap(1.0, 0.0, cx, cy));
    EXPECT_EQ(g.at(cx, cy), LETHAL_OBSTACLE);  // single noise hit reads obstacle
  }
  {
    std::vector<StaticKeyframe> kfs{bad, good(), good(), good()};
    CostmapGrid g;
    ASSERT_TRUE(layer.buildFromKeyframes(kfs, gl, p, g));
    unsigned int cx = 0, cy = 0;
    ASSERT_TRUE(g.worldToMap(1.0, 0.0, cx, cy));
    EXPECT_EQ(g.at(cx, cy), FREE_SPACE);  // voted out by repeated free crossings
  }
}

TEST(StaticLayer, NormalGatePromotesVerticalFaceFromKeyframe) {
  // A return at ground level (FREE by height) with a vertical normal is a wall
  // base -> promoted to OBSTACLE by the gate.
  GroundLookup gl{nullptr, 0.0, true};
  StaticLayer layer;
  StaticLayerParams p;
  p.resolution = 0.1;
  p.margin = 0.5;
  p.walkable_layers = 1;
  p.obstacle_max_layers = 12;
  p.xy_clip_pct = 0.0;
  p.use_normals = true;
  p.ground_normal_angle = 0.7;
  p.vertical_normal_angle = 1.0;
  p.normal_flat_grace_layers = 2;

  StaticKeyframe k;
  k.origin = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  k.points.emplace_back(0.5f, 0.0f, 0.0f);
  k.normals.emplace_back(1.0f, 0.0f, 0.0f);  // vertical surface (horizontal normal)
  CostmapGrid g;
  ASSERT_TRUE(layer.buildFromKeyframes({k}, gl, p, g));
  unsigned int mx = 0, my = 0;
  ASSERT_TRUE(g.worldToMap(0.5, 0.0, mx, my));
  EXPECT_EQ(g.at(mx, my), LETHAL_OBSTACLE);
}

TEST(StaticLayer, NormalGateDemotesFlatDriftFromKeyframe) {
  // A flat return a couple layers above ground (OBSTACLE by height) is drifted
  // floor -> demoted to FREE by the gate.
  GroundLookup gl{nullptr, 0.0, true};
  StaticLayer layer;
  StaticLayerParams p;
  p.resolution = 0.1;
  p.margin = 0.5;
  p.walkable_layers = 1;
  p.obstacle_max_layers = 12;
  p.xy_clip_pct = 0.0;
  p.use_normals = true;
  p.ground_normal_angle = 0.7;
  p.vertical_normal_angle = 1.0;
  p.normal_flat_grace_layers = 2;

  StaticKeyframe k;
  k.origin = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  k.points.emplace_back(0.5f, 0.0f, 0.2f);   // offset 2 -> OBSTACLE by height
  k.normals.emplace_back(0.0f, 0.0f, 1.0f);  // flat (horizontal) surface
  CostmapGrid g;
  ASSERT_TRUE(layer.buildFromKeyframes({k}, gl, p, g));
  unsigned int mx = 0, my = 0;
  ASSERT_TRUE(g.worldToMap(0.5, 0.0, mx, my));
  EXPECT_EQ(g.at(mx, my), FREE_SPACE);
}

TEST(StaticLayer, RaycastStopsAtObstacleNoClearBehind) {
  // A beam to a far floor return passes over a near obstacle (a table edge). The
  // occluded cell behind the obstacle must NOT be cleared free (stays unknown).
  GroundLookup gl{nullptr, 0.0, true};  // flat ground 0
  StaticLayer layer;
  StaticLayerParams p;
  p.resolution = 0.1;
  p.margin = 0.95;  // half-cell offset: keep coords mid-cell (avoid float boundary)
  p.walkable_layers = 1;
  p.obstacle_max_layers = 20;
  p.phantom_drop = 0.1;
  p.xy_clip_pct = 0.0;

  StaticKeyframe kf;
  kf.origin = Eigen::Vector3f(0.05f, 0.05f, 1.0f);  // lidar at 1.0 m
  kf.points.emplace_back(1.0f, 0.05f, 0.5f);   // obstacle (in band: below lidar)
  kf.points.emplace_back(2.0f, 0.05f, 0.0f);   // far floor return beyond it
  CostmapGrid g;
  ASSERT_TRUE(layer.buildFromKeyframes({kf}, gl, p, g));

  unsigned int fx = 0, fy = 0;
  ASSERT_TRUE(g.worldToMap(0.5, 0.05, fx, fy));
  EXPECT_EQ(g.at(fx, fy), FREE_SPACE);  // before the obstacle: cleared
  unsigned int wx = 0, wy = 0;
  ASSERT_TRUE(g.worldToMap(1.0, 0.05, wx, wy));
  EXPECT_EQ(g.at(wx, wy), LETHAL_OBSTACLE);  // the obstacle
  unsigned int bx = 0, by = 0;
  ASSERT_TRUE(g.worldToMap(1.5, 0.05, bx, by));
  EXPECT_EQ(g.at(bx, by), NO_INFORMATION);  // occluded behind: NOT cleared
}

TEST(StaticLayer, AboveLidarPlaneIgnored) {
  // Band top = lidar plane (origin z) - phantom_drop: below it -> obstacle, above
  // it -> ignored (overhead the robot walks under).
  GroundLookup gl{nullptr, 0.0, true};
  StaticLayer layer;
  StaticLayerParams p;
  p.resolution = 0.1;
  p.margin = 0.55;  // half-cell offset: keep coords mid-cell (avoid float boundary)
  p.walkable_layers = 1;
  p.obstacle_max_layers = 20;  // safety cap well above the lidar plane
  p.phantom_drop = 0.1;
  p.xy_clip_pct = 0.0;

  StaticKeyframe kf;
  kf.origin = Eigen::Vector3f(0.05f, 0.05f, 1.0f);  // lidar plane ~0.9 m
  kf.points.emplace_back(0.5f, 0.05f, 0.6f);   // below the lidar plane -> obstacle
  kf.points.emplace_back(0.7f, 0.05f, 1.5f);   // above the lidar plane -> ignored
  CostmapGrid g;
  ASSERT_TRUE(layer.buildFromKeyframes({kf}, gl, p, g));

  unsigned int ax = 0, ay = 0;
  ASSERT_TRUE(g.worldToMap(0.5, 0.05, ax, ay));
  EXPECT_EQ(g.at(ax, ay), LETHAL_OBSTACLE);  // below lidar -> obstacle
  unsigned int cx = 0, cy = 0;
  ASSERT_TRUE(g.worldToMap(0.7, 0.05, cx, cy));
  EXPECT_NE(g.at(cx, cy), LETHAL_OBSTACLE);  // above lidar -> not an obstacle
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
