// NavPointStore tests: nearest-keyframe anchoring, resolution, anchoring invariant
// under re-optimization, duplicate/overwrite, missing anchor, max_dist rejection.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "g1_map_manager/core/map_types.h"
#include "g1_map_manager/core/nav_point_store.h"

using namespace g1_map_manager;

namespace {

KeyframePoseRec kf(uint32_t id, double x, double y, double yaw = 0.0) {
  KeyframePoseRec k;
  k.id = id;
  k.pose = Eigen::Isometry3d::Identity();
  k.pose.translation() << x, y, 0.0;
  k.pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return k;
}

Pose3 worldXY(double x, double y) {
  Pose3 p;
  p.x = x;
  p.y = y;
  return p;
}

}  // namespace

TEST(NavPointStore, AddAnchorsToNearestKeyframe) {
  NavPointStore s;
  const std::vector<KeyframePoseRec> kfs = {kf(0, 0, 0), kf(1, 10, 0)};
  NavPointRec rec;
  std::string err;
  ASSERT_TRUE(s.add("base", worldXY(10.5, 0.0), kfs, 5.0, false, rec, err)) << err;
  EXPECT_EQ(rec.anchor_id, 1u);  // nearest is kf 1 (0.5 m)
  // rel = inverse(kf1) * world = translation (0.5, 0, 0).
  EXPECT_NEAR(rec.rel.translation().x(), 0.5, 1e-9);
  EXPECT_NEAR(rec.rel.translation().y(), 0.0, 1e-9);
}

TEST(NavPointStore, ResolveReproducesWorldPose) {
  NavPointStore s;
  const std::vector<KeyframePoseRec> kfs = {kf(0, 0, 0), kf(1, 10, 0)};
  NavPointRec rec;
  std::string err;
  ASSERT_TRUE(s.add("base", worldXY(10.5, 2.0), kfs, 5.0, false, rec, err));
  Pose3 w;
  ASSERT_TRUE(s.resolve("base", kfs, w));
  EXPECT_NEAR(w.x, 10.5, 1e-9);
  EXPECT_NEAR(w.y, 2.0, 1e-9);
}

TEST(NavPointStore, ResolveTracksReoptimizedAnchor) {
  NavPointStore s;
  std::vector<KeyframePoseRec> kfs = {kf(0, 0, 0), kf(1, 10, 0)};
  NavPointRec rec;
  std::string err;
  ASSERT_TRUE(s.add("base", worldXY(10.5, 0.0), kfs, 5.0, false, rec, err));

  // Loop closure re-optimizes the anchor keyframe (moved + rotated).
  kfs[1] = kf(1, 20.0, 5.0, 1.2);
  Pose3 w;
  ASSERT_TRUE(s.resolve("base", kfs, w));
  const Eigen::Isometry3d expected = kfs[1].pose * rec.rel;  // anchor * rel
  EXPECT_NEAR(w.x, expected.translation().x(), 1e-9);
  EXPECT_NEAR(w.y, expected.translation().y(), 1e-9);
}

TEST(NavPointStore, DuplicateRejectedUnlessOverwrite) {
  NavPointStore s;
  const std::vector<KeyframePoseRec> kfs = {kf(0, 0, 0)};
  NavPointRec rec;
  std::string err;
  ASSERT_TRUE(s.add("base", worldXY(0.1, 0.0), kfs, 5.0, false, rec, err));
  EXPECT_FALSE(s.add("base", worldXY(0.2, 0.0), kfs, 5.0, false, rec, err));
  EXPECT_TRUE(s.add("base", worldXY(0.2, 0.0), kfs, 5.0, true, rec, err)) << err;
  EXPECT_EQ(s.all().size(), 1u);
}

TEST(NavPointStore, ResolveFalseWhenAnchorMissing) {
  NavPointStore s;
  std::vector<KeyframePoseRec> kfs = {kf(7, 0, 0)};
  NavPointRec rec;
  std::string err;
  ASSERT_TRUE(s.add("base", worldXY(0.1, 0.0), kfs, 5.0, false, rec, err));
  // The anchor keyframe (id 7) is gone (e.g. trimmed).
  std::vector<KeyframePoseRec> without = {kf(0, 0, 0)};
  Pose3 w;
  EXPECT_FALSE(s.resolve("base", without, w));
}

TEST(NavPointStore, AddBeyondMaxDistFails) {
  NavPointStore s;
  const std::vector<KeyframePoseRec> kfs = {kf(0, 0, 0)};
  NavPointRec rec;
  std::string err;
  EXPECT_FALSE(s.add("far", worldXY(100.0, 0.0), kfs, 5.0, false, rec, err));
  EXPECT_FALSE(err.empty());
}

TEST(NavPointStore, SerializeRoundTrip) {
  NavPointStore s;
  s.setMapName("floor1");
  const std::vector<KeyframePoseRec> kfs = {kf(0, 0, 0), kf(1, 10, 0, 0.5)};
  NavPointRec rec;
  std::string err;
  ASSERT_TRUE(s.add("base", worldXY(0.3, 0.0), kfs, 5.0, false, rec, err));
  ASSERT_TRUE(s.add("delivery", worldXY(10.2, 0.4), kfs, 5.0, false, rec, err));

  NavPointStore s2;
  s2.load(s.serialize());
  EXPECT_EQ(s2.mapName(), "floor1");
  ASSERT_EQ(s2.all().size(), 2u);
  // Resolved world poses match across the round-trip.
  Pose3 a, b;
  ASSERT_TRUE(s.resolve("delivery", kfs, a));
  ASSERT_TRUE(s2.resolve("delivery", kfs, b));
  EXPECT_NEAR(a.x, b.x, 1e-9);
  EXPECT_NEAR(a.y, b.y, 1e-9);
}

TEST(NavPointStore, UndoRedoRoundTrip) {
  NavPointStore s;
  const std::vector<KeyframePoseRec> kfs = {kf(0, 0, 0)};
  NavPointRec rec;
  std::string err;
  ASSERT_TRUE(s.add("a", worldXY(0.1, 0.0), kfs, 5.0, false, rec, err)) << err;
  ASSERT_TRUE(s.add("b", worldXY(0.2, 0.0), kfs, 5.0, false, rec, err)) << err;
  EXPECT_TRUE(s.has("a"));
  EXPECT_TRUE(s.has("b"));

  ASSERT_TRUE(s.undo());  // undo add "b"
  EXPECT_TRUE(s.has("a"));
  EXPECT_FALSE(s.has("b"));

  ASSERT_TRUE(s.redo());  // redo add "b"
  EXPECT_TRUE(s.has("b"));
}

TEST(NavPointStore, UndoCoversRemove) {
  NavPointStore s;
  const std::vector<KeyframePoseRec> kfs = {kf(0, 0, 0)};
  NavPointRec rec;
  std::string err;
  ASSERT_TRUE(s.add("a", worldXY(0.1, 0.0), kfs, 5.0, false, rec, err)) << err;
  ASSERT_TRUE(s.remove("a"));
  EXPECT_FALSE(s.has("a"));
  ASSERT_TRUE(s.undo());  // undo the remove
  EXPECT_TRUE(s.has("a"));
}

TEST(NavPointStore, UndoEmptyReturnsFalse) {
  NavPointStore s;
  EXPECT_FALSE(s.undo());
  EXPECT_FALSE(s.redo());
}

TEST(NavPointStore, LoadClearsUndoHistory) {
  NavPointStore s;
  const std::vector<KeyframePoseRec> kfs = {kf(0, 0, 0)};
  NavPointRec rec;
  std::string err;
  ASSERT_TRUE(s.add("a", worldXY(0.1, 0.0), kfs, 5.0, false, rec, err)) << err;
  s.load("");  // reload wipes history
  EXPECT_FALSE(s.undo());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
