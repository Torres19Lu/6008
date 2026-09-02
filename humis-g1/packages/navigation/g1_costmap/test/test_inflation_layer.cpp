// InflationLayer unit tests: inscribed band, monotonic radial falloff, cutoff.

#include <gtest/gtest.h>

#include <cstdint>

#include "g1_costmap/core/costmap_grid.h"
#include "g1_costmap/layers/inflation_layer.h"

using namespace g1_costmap;

TEST(InflationLayer, RadialProfileFromSingleObstacle) {
  CostmapGrid g(0.1, 0.0, 0.0, 41, 41);  // center cell (20,20)
  g.setCost(20, 20, LETHAL_OBSTACLE);

  InflationLayer infl;
  InflationParams p;
  p.robot_radius = 0.2;       // 2 cells inscribed
  p.inflation_radius = 0.5;   // 5 cells
  p.cost_scaling_factor = 3.0;
  infl.inflate(g, p);

  EXPECT_EQ(g.at(20, 20), LETHAL_OBSTACLE);        // obstacle preserved
  EXPECT_EQ(g.at(21, 20), INSCRIBED_INFLATED);     // 0.1 m <= robot_radius
  EXPECT_EQ(g.at(22, 20), INSCRIBED_INFLATED);     // 0.2 m == robot_radius

  // Monotonic non-increasing cost moving away from the obstacle.
  int prev = 256;
  for (int d = 1; d <= 6; ++d) {
    const int c = g.at(20 + d, 20);
    EXPECT_LE(c, prev) << "non-monotonic at distance " << d;
    prev = c;
  }

  EXPECT_GT(g.at(25, 20), FREE_SPACE);   // 0.5 m, at the inflation edge
  EXPECT_EQ(g.at(26, 20), FREE_SPACE);   // 0.6 m, beyond inflation_radius
}

TEST(InflationLayer, NoObstacleLeavesGridFree) {
  CostmapGrid g(0.1, 0.0, 0.0, 10, 10);
  InflationLayer infl;
  InflationParams p;
  infl.inflate(g, p);
  for (auto c : g.data()) EXPECT_EQ(c, FREE_SPACE);
}

TEST(InflationLayer, ObstacleCellsAreNeverDowngraded) {
  // Two obstacles within each other's inscribed band: inflation must NOT lower
  // either obstacle cell to inscribed, and the cell between them takes the
  // nearer (higher) cost (lethal), not a farther obstacle's lower cost.
  CostmapGrid g(0.1, 0.0, 0.0, 30, 10);
  g.setCost(10, 5, LETHAL_OBSTACLE);
  g.setCost(12, 5, LETHAL_OBSTACLE);
  InflationLayer infl;
  InflationParams p;
  p.robot_radius = 0.3;
  p.inflation_radius = 0.8;
  p.cost_scaling_factor = 3.0;
  infl.inflate(g, p);
  EXPECT_EQ(g.at(10, 5), LETHAL_OBSTACLE);  // obstacle preserved
  EXPECT_EQ(g.at(12, 5), LETHAL_OBSTACLE);  // obstacle preserved (not downgraded)
  EXPECT_EQ(g.at(11, 5), INSCRIBED_INFLATED);  // 0.1 m from both -> inscribed
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
