// Overlay model tests: parse/serialize round-trip, point-in-polygon, empty inputs.

#include <gtest/gtest.h>

#include <array>
#include <string>

#include "g1_map_manager/core/overlay.h"

using namespace g1_map_manager;

TEST(Overlay, ObstacleRoundTrip) {
  Overlay o;
  o.deleted_voxels = {{1.0, 2.0, 0.5}, {-3.0, 4.0, 0.0}};
  const Overlay r = parseOverlay(serializeObstacle(o), "", "");
  EXPECT_EQ(r.deleted_voxels, o.deleted_voxels);
}

TEST(Overlay, TrimRoundTrip) {
  Overlay o;
  o.suppressed_keyframes = {3, 7, 11};
  o.crops = {{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}},
             {{5.0, 5.0}, {6.0, 5.0}, {6.0, 6.0}}};
  const Overlay r = parseOverlay("", serializeTrim(o), "");
  EXPECT_EQ(r.suppressed_keyframes, o.suppressed_keyframes);
  EXPECT_EQ(r.crops, o.crops);
}

TEST(Overlay, MetadataRoundTrip) {
  Overlay o;
  o.metadata = {{"name", "Lab Floor 2"}, {"description", "north wing"}};
  const Overlay r = parseOverlay("", "", serializeMetadata(o));
  EXPECT_EQ(r.metadata, o.metadata);
}

TEST(Overlay, EmptyInputsYieldEmptyOverlay) {
  const Overlay r = parseOverlay("", "", "");
  EXPECT_TRUE(r.deleted_voxels.empty());
  EXPECT_TRUE(r.suppressed_keyframes.empty());
  EXPECT_TRUE(r.crops.empty());
  EXPECT_TRUE(r.metadata.empty());
}

TEST(Overlay, InsidePolygonEvenOdd) {
  const std::vector<std::array<double, 2>> square = {
      {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
  EXPECT_TRUE(insidePolygon({1.0, 1.0}, square));
  EXPECT_FALSE(insidePolygon({3.0, 1.0}, square));
  EXPECT_FALSE(insidePolygon({-1.0, 1.0}, square));
  EXPECT_FALSE(insidePolygon({1.0, 1.0}, {{0.0, 0.0}, {1.0, 0.0}}));  // degenerate
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
