#pragma once
// map_editor.h -- staged, non-destructive map content editing over the edit overlay
// (obstacle-mask deletions, keyframe trims + crop polygons, metadata overrides), with
// undo/redo. The version snapshot is never touched; only the edits/*.yaml change.
// ROS-free; the node adds /clicked_point capture + file I/O. Mirrors GatewayEditor.

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "g1_map_manager/core/overlay.h"

namespace g1_map_manager {

struct MapEditArgs {
  std::string map_name, key, value;
  uint32_t keyframe_id = 0;
  std::vector<std::array<double, 3>> points3;  // obstacle voxels (delete_obstacle)
  std::vector<std::array<double, 2>> points2;  // crop polygon vertices (crop)
};

class MapEditor {
 public:
  void load(const Overlay& existing);

  // command: delete_obstacle|trim_keyframe|crop|set_metadata|status|undo|redo|
  //          discard|save. Returns false (with msg) on an invalid command.
  bool apply(const std::string& command, const MapEditArgs& args, std::string& msg);

  const Overlay& overlay() const { return overlay_; }
  bool dirty() const { return dirty_; }
  void markSaved() { loaded_ = overlay_; dirty_ = false; }

 private:
  void pushUndo();
  Overlay loaded_, overlay_;
  std::vector<Overlay> undo_, redo_;
  bool dirty_ = false;
};

}  // namespace g1_map_manager
