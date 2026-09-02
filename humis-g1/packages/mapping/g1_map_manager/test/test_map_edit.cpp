// MapEditor tests: each capability mutates the overlay, undo/redo, discard reverts
// to the loaded state, and save marks the overlay persisted.

#include <gtest/gtest.h>

#include <string>

#include "g1_map_manager/core/map_editor.h"
#include "g1_map_manager/core/overlay.h"

using namespace g1_map_manager;

TEST(MapEditor, DeleteObstacleTrimCropMetadata) {
  MapEditor ed;
  ed.load(Overlay{});
  std::string msg;

  MapEditArgs del;
  del.points3 = {{1.0, 2.0, 0.5}};
  ASSERT_TRUE(ed.apply("delete_obstacle", del, msg)) << msg;
  EXPECT_EQ(ed.overlay().deleted_voxels.size(), 1u);

  MapEditArgs trim;
  trim.keyframe_id = 7;
  ASSERT_TRUE(ed.apply("trim_keyframe", trim, msg)) << msg;
  EXPECT_EQ(ed.overlay().suppressed_keyframes, (std::vector<std::uint32_t>{7}));

  MapEditArgs crop;
  crop.points2 = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  ASSERT_TRUE(ed.apply("crop", crop, msg)) << msg;
  ASSERT_EQ(ed.overlay().crops.size(), 1u);
  EXPECT_EQ(ed.overlay().crops[0].size(), 4u);

  MapEditArgs meta;
  meta.key = "description";
  meta.value = "north wing";
  ASSERT_TRUE(ed.apply("set_metadata", meta, msg)) << msg;
  EXPECT_EQ(ed.overlay().metadata.at("description"), "north wing");
  EXPECT_TRUE(ed.dirty());
}

TEST(MapEditor, UndoRedo) {
  MapEditor ed;
  ed.load(Overlay{});
  std::string msg;
  MapEditArgs trim;
  trim.keyframe_id = 3;
  ed.apply("trim_keyframe", trim, msg);
  ASSERT_EQ(ed.overlay().suppressed_keyframes.size(), 1u);

  ASSERT_TRUE(ed.apply("undo", {}, msg));
  EXPECT_TRUE(ed.overlay().suppressed_keyframes.empty());
  ASSERT_TRUE(ed.apply("redo", {}, msg));
  EXPECT_EQ(ed.overlay().suppressed_keyframes.size(), 1u);
}

TEST(MapEditor, DiscardRevertsToLoaded) {
  Overlay loaded;
  loaded.suppressed_keyframes = {1};
  MapEditor ed;
  ed.load(loaded);
  std::string msg;
  MapEditArgs trim;
  trim.keyframe_id = 9;
  ed.apply("trim_keyframe", trim, msg);
  ASSERT_EQ(ed.overlay().suppressed_keyframes.size(), 2u);

  ASSERT_TRUE(ed.apply("discard", {}, msg));
  EXPECT_EQ(ed.overlay().suppressed_keyframes, (std::vector<std::uint32_t>{1}));
  EXPECT_FALSE(ed.dirty());
}

TEST(MapEditor, SaveMarksPersisted) {
  MapEditor ed;
  ed.load(Overlay{});
  std::string msg;
  MapEditArgs meta;
  meta.key = "name";
  meta.value = "Lab";
  ed.apply("set_metadata", meta, msg);
  ASSERT_TRUE(ed.dirty());
  ASSERT_TRUE(ed.apply("save", {}, msg));
  // The node writes the files on save; markSaved clears dirty.
  ed.markSaved();
  EXPECT_FALSE(ed.dirty());
  // serialized overlay carries the edit.
  const Overlay r = parseOverlay("", "", serializeMetadata(ed.overlay()));
  EXPECT_EQ(r.metadata.at("name"), "Lab");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
