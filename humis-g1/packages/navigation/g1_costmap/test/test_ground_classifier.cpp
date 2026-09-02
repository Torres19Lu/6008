// ground_classifier unit tests: layer snapping, the 3-state band, normal gate.

#include <gtest/gtest.h>

#include "g1_costmap/core/ground_classifier.h"

using namespace g1_costmap;

TEST(GroundClassifier, LayerOfFloorsNegatives) {
  EXPECT_EQ(layerOf(0.05, 0.1), 0);
  EXPECT_EQ(layerOf(0.15, 0.1), 1);
  EXPECT_EQ(layerOf(-0.001, 0.1), -1);  // just below ground is layer -1, not 0
  EXPECT_EQ(layerOf(-0.1, 0.1), -1);
}

TEST(GroundClassifier, Classify3Bands) {
  const int walkable = 1, obstacle_max = 12;
  EXPECT_EQ(classify3(0, walkable, obstacle_max), Tier3::FREE);      // at ground
  EXPECT_EQ(classify3(1, walkable, obstacle_max), Tier3::FREE);      // walkable margin
  EXPECT_EQ(classify3(-5, walkable, obstacle_max), Tier3::FREE);     // below ground -> free
  EXPECT_EQ(classify3(2, walkable, obstacle_max), Tier3::OBSTACLE);  // just above margin
  EXPECT_EQ(classify3(12, walkable, obstacle_max), Tier3::OBSTACLE); // band top
  EXPECT_EQ(classify3(13, walkable, obstacle_max), Tier3::IGNORED);  // above band (ceiling)
}

TEST(GroundClassifier, NormalAngleFromUp) {
  bool ok = false;
  EXPECT_NEAR(normalAngleFromUp(0, 0, 1, ok), 0.0, 1e-6);  // horizontal surface
  EXPECT_TRUE(ok);
  EXPECT_NEAR(normalAngleFromUp(1, 0, 0, ok), M_PI / 2, 1e-6);  // vertical surface
  EXPECT_TRUE(ok);
  EXPECT_NEAR(normalAngleFromUp(0, 0, -1, ok), 0.0, 1e-6);  // sign-agnostic
  normalAngleFromUp(0, 0, 0, ok);  // degenerate
  EXPECT_FALSE(ok);
}

TEST(GroundClassifier, NormalGateFallsBackWithoutNormal) {
  const int walk = 1, omax = 12;
  const double ga = 0.7, va = 1.05;
  // has_normal=false -> identical to classify3.
  EXPECT_EQ(classify3WithNormal(5, walk, omax, false, 0, ga, va, 2),
            Tier3::OBSTACLE);
  EXPECT_EQ(classify3WithNormal(0, walk, omax, false, 0, ga, va, 2), Tier3::FREE);
}

TEST(GroundClassifier, NormalGateDemotesFlatFloorDrift) {
  const int walk = 1, omax = 12;
  const double ga = 0.7, va = 1.05, flat = 0.0;  // angle 0 = flat
  // offset 2 is OBSTACLE by height, but a flat surface within walkable+grace(2)
  // is drifted floor -> FREE.
  EXPECT_EQ(classify3(2, walk, omax), Tier3::OBSTACLE);
  EXPECT_EQ(classify3WithNormal(2, walk, omax, true, flat, ga, va, 2), Tier3::FREE);
  // A flat surface well above the band (a real table top) is NOT demoted.
  EXPECT_EQ(classify3WithNormal(7, walk, omax, true, flat, ga, va, 2),
            Tier3::OBSTACLE);
}

TEST(GroundClassifier, BandTopFollowsLidarPlane) {
  const double res = 0.1, drop = 0.1;
  const int safety = 20;  // ground + 2.0 m fixed cap
  // Lidar at 1.0 m, ground 0 -> band top = floor((1.0-0.1)/0.1) ~ 8-9 layers,
  // well under the safety cap.
  const int top = bandTopLayers(1.0, 0.0, drop, safety, res);
  EXPECT_GE(top, 7);
  EXPECT_LE(top, 9);
  EXPECT_LT(top, safety);  // the lidar plane bites before the safety cap
  // A point 6 layers up (0.6 m, below the lidar plane) is in band -> OBSTACLE.
  EXPECT_EQ(classify3(6, 1, top), Tier3::OBSTACLE);
  // A point 12 layers up (1.2 m, above the lidar plane) is IGNORED (ceiling).
  EXPECT_EQ(classify3(12, 1, top), Tier3::IGNORED);
}

TEST(GroundClassifier, BandTopSafetyCapWhenLidarHigh) {
  const double res = 0.1, drop = 0.1;
  const int safety = 20;
  // Lidar drifted to 5 m: the fixed safety cap (20 layers) bounds the band.
  EXPECT_EQ(bandTopLayers(5.0, 0.0, drop, safety, res), safety);
}

TEST(GroundClassifier, BandTopFallsBackOnDegenerateLidar) {
  const double res = 0.1, drop = 0.1;
  const int safety = 12;
  // Lidar at/below ground (bad pose z) -> fall back to the fixed cap, not an empty band.
  EXPECT_EQ(bandTopLayers(0.0, 0.0, drop, safety, res), safety);
}

TEST(GroundClassifier, NormalGatePromotesVerticalFaceOnFloor) {
  const int walk = 1, omax = 12;
  const double ga = 0.7, va = 1.05, vert = M_PI / 2;  // angle pi/2 = vertical
  // offset 0 is FREE by height, but a vertical face at the ground layer is an
  // obstacle base -> OBSTACLE.
  EXPECT_EQ(classify3(0, walk, omax), Tier3::FREE);
  EXPECT_EQ(classify3WithNormal(0, walk, omax, true, vert, ga, va, 2),
            Tier3::OBSTACLE);
  // Below ground stays FREE even if the (noisy) normal looks vertical.
  EXPECT_EQ(classify3WithNormal(-3, walk, omax, true, vert, ga, va, 2),
            Tier3::FREE);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
