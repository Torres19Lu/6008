// MapLineage IO tests + read-only flat parse of the backend manifest.

#include <gtest/gtest.h>

#include "g1_map_manager/core/map_lineage.h"

using namespace g1_map_manager;

TEST(MapLineage, RoundTripsAllFields) {
  MapLineage m;
  m.name = "eee-b4";
  m.created_at = "2026-06-24T09:42:00Z";
  m.label = "extend west wing";
  m.reason = "incremental";
  m.parent = "20260624-093000-pre-incremental";

  const MapLineage r = parseLineage(serializeLineage(m));
  EXPECT_EQ(r.name, "eee-b4");
  EXPECT_EQ(r.created_at, "2026-06-24T09:42:00Z");
  EXPECT_EQ(r.label, "extend west wing");
  EXPECT_EQ(r.reason, "incremental");
  EXPECT_EQ(r.parent, "20260624-093000-pre-incremental");
}

TEST(MapLineage, ParsesEmptyAsDefault) {
  const MapLineage r = parseLineage("");
  EXPECT_TRUE(r.name.empty());
  EXPECT_TRUE(r.parent.empty());
}

TEST(MapLineage, ReadsBackendManifestNumKeyframesFlat) {
  // Verbatim backend schema (g1_slam_backend/src/backend.cpp): flat key:value.
  const std::string manifest =
      "# g1_slam_backend map. Inspectable and re-optimizable.\n"
      "format_version: 1\n"
      "num_keyframes: 560\n"
      "keyframe_voxel: 0.25\n"
      "sc_num_rings: 20\n";
  EXPECT_EQ(parseBackendNumKeyframes(manifest), 560u);
  EXPECT_EQ(parseBackendNumKeyframes(""), 0u);
  EXPECT_EQ(parseBackendNumKeyframes("name: x\n"), 0u);  // key absent -> 0
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
