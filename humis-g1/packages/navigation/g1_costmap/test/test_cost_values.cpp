// Unit tests for the internal-cost -> OccupancyGrid mapping. The split that makes
// INSCRIBED (253) distinct from LETHAL (254) is the foundation of the close-approach
// + smooth-gradient work: 254 -> 100 (real obstacle), 253 -> 99 (center-blocked),
// inflation band -> [1,98], monotonic non-decreasing.
#include <gtest/gtest.h>

#include "g1_costmap/core/cost_values.h"

using g1_costmap::costToOccupancy;
using g1_costmap::FREE_SPACE;
using g1_costmap::INSCRIBED_INFLATED;
using g1_costmap::LETHAL_OBSTACLE;
using g1_costmap::NO_INFORMATION;

TEST(CostValues, AnchorsMapToDistinctLevels) {
  EXPECT_EQ(costToOccupancy(FREE_SPACE), 0);
  EXPECT_EQ(costToOccupancy(NO_INFORMATION), -1);
  EXPECT_EQ(costToOccupancy(INSCRIBED_INFLATED), 99);   // 253 -> inscribed
  EXPECT_EQ(costToOccupancy(LETHAL_OBSTACLE), 100);     // 254 -> lethal
}

TEST(CostValues, InflationBandStaysBelowInscribed) {
  // Every inflation cost (1..252) maps into [1,98], strictly below inscribed(99).
  for (int c = 1; c < INSCRIBED_INFLATED; ++c) {
    const int v = costToOccupancy(static_cast<std::uint8_t>(c));
    EXPECT_GE(v, 1);
    EXPECT_LE(v, 98) << "inflation cost " << c << " leaked into the inscribed band";
  }
}

TEST(CostValues, MonotonicNonDecreasing) {
  int prev = costToOccupancy(0);
  for (int c = 1; c <= 255; ++c) {
    const int v = costToOccupancy(static_cast<std::uint8_t>(c));
    if (c == NO_INFORMATION) continue;  // -1 is out of the monotone chain
    EXPECT_GE(v, prev) << "non-monotonic at cost " << c;
    prev = v;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
