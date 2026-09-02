#include <gtest/gtest.h>
#include "reloc_quality.h"

using g1_slam_backend::Backend;
using g1_slam_backend::toRelocQualityMsg;

TEST(RelocQuality, MapsAllFieldsAndAcceptedTracksFound) {
  Backend::RelocResult r;
  r.found = true; r.fitness = 0.12; r.inlier_ratio = 0.77;
  r.sc_distance = 0.21; r.match_id = 42;
  const auto q = toRelocQualityMsg(r);
  EXPECT_DOUBLE_EQ(q.fitness, 0.12);
  EXPECT_DOUBLE_EQ(q.inlier_ratio, 0.77);
  EXPECT_DOUBLE_EQ(q.sc_distance, 0.21);
  EXPECT_EQ(q.match_id, 42u);
  EXPECT_TRUE(q.accepted);

  Backend::RelocResult miss;            // diagnostics filled even when not found
  miss.found = false; miss.inlier_ratio = 0.1;
  EXPECT_FALSE(toRelocQualityMsg(miss).accepted);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
