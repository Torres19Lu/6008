// GroundSurface unit tests: per-cell lowest, robust median (outlier rejection),
// min-points trust, unobserved -> nullopt.

#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Core>

#include "g1_costmap/core/ground_surface.h"

using namespace g1_costmap;

TEST(GroundSurface, LowestInformsGround) {
  GroundSurface gs(0.1, 0.0, 1);  // radius 0 -> ground is the cell's own lowest
  std::vector<Eigen::Vector3f> pts;
  pts.emplace_back(0.05f, 0.05f, 0.5f);
  pts.emplace_back(0.05f, 0.05f, 0.2f);  // lowest
  gs.update(pts);
  const auto g = gs.groundZAt(0.05, 0.05);
  ASSERT_TRUE(g.has_value());
  EXPECT_NEAR(*g, 0.2, 1e-6);
}

TEST(GroundSurface, MedianRejectsLowOutlier) {
  // A 3x3 block of cells at z=0.5; the centre cell also has a far-below outlier.
  // The robust median over the neighbourhood must ignore the outlier.
  GroundSurface gs(0.1, 0.25, 1);
  std::vector<Eigen::Vector3f> pts;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      pts.emplace_back(0.05f + 0.1f * i, 0.05f + 0.1f * j, 0.5f);
    }
  }
  pts.emplace_back(0.15f, 0.15f, -5.0f);  // outlier in the centre cell
  gs.update(pts);
  const auto g = gs.groundZAt(0.15, 0.15);
  ASSERT_TRUE(g.has_value());
  EXPECT_NEAR(*g, 0.5, 1e-6);  // median of the neighbourhood, not -5
}

TEST(GroundSurface, BelowMinPointsIsUntrusted) {
  GroundSurface gs(0.1, 0.0, 3);  // need 3 returns
  std::vector<Eigen::Vector3f> pts;
  pts.emplace_back(0.05f, 0.05f, 0.2f);  // only 1
  gs.update(pts);
  EXPECT_FALSE(gs.groundZAt(0.05, 0.05).has_value());
}

TEST(GroundSurface, UnobservedCellIsNullopt) {
  GroundSurface gs(0.1, 0.3, 1);
  std::vector<Eigen::Vector3f> pts;
  pts.emplace_back(0.05f, 0.05f, 0.0f);
  gs.update(pts);
  EXPECT_FALSE(gs.groundZAt(5.0, 5.0).has_value());  // far from any return
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
