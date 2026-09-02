// map_editor.cpp -- staged non-destructive map content editing.

#include "g1_map_manager/core/map_editor.h"

namespace g1_map_manager {

void MapEditor::load(const Overlay& existing) {
  loaded_ = existing;
  overlay_ = existing;
  undo_.clear();
  redo_.clear();
  dirty_ = false;
}

void MapEditor::pushUndo() {
  undo_.push_back(overlay_);
  redo_.clear();
}

bool MapEditor::apply(const std::string& command, const MapEditArgs& args,
                      std::string& msg) {
  if (command == "delete_obstacle") {
    pushUndo();
    for (const auto& p : args.points3) overlay_.deleted_voxels.push_back(p);
    dirty_ = true;
    msg = "deleted obstacle voxel(s)";
    return true;
  }
  if (command == "trim_keyframe") {
    pushUndo();
    overlay_.suppressed_keyframes.push_back(args.keyframe_id);
    dirty_ = true;
    msg = "suppressed keyframe " + std::to_string(args.keyframe_id);
    return true;
  }
  if (command == "crop") {
    pushUndo();
    overlay_.crops.push_back(args.points2);
    dirty_ = true;
    msg = "added crop polygon";
    return true;
  }
  if (command == "set_metadata") {
    pushUndo();
    overlay_.metadata[args.key] = args.value;
    dirty_ = true;
    msg = "set metadata " + args.key;
    return true;
  }
  if (command == "status") {
    msg = std::to_string(overlay_.deleted_voxels.size()) + " voxels, " +
          std::to_string(overlay_.suppressed_keyframes.size()) + " trims, " +
          std::to_string(overlay_.crops.size()) + " crops, " +
          std::to_string(overlay_.metadata.size()) + " metadata" +
          (dirty_ ? " (unsaved)" : "");
    return true;
  }
  if (command == "discard") {
    overlay_ = loaded_;
    undo_.clear();
    redo_.clear();
    dirty_ = false;
    msg = "discarded unsaved edits";
    return true;
  }
  if (command == "save") {
    // The node serializes + writes the three overlay files + backups, then calls
    // markSaved(); dirty stays until then so the node knows to write.
    msg = dirty_ ? "ready to write overlay" : "no unsaved edits";
    return true;
  }
  if (command == "undo") {
    if (undo_.empty()) { msg = "nothing to undo"; return false; }
    redo_.push_back(overlay_);
    overlay_ = undo_.back();
    undo_.pop_back();
    dirty_ = true;
    msg = "undone";
    return true;
  }
  if (command == "redo") {
    if (redo_.empty()) { msg = "nothing to redo"; return false; }
    undo_.push_back(overlay_);
    overlay_ = redo_.back();
    redo_.pop_back();
    dirty_ = true;
    msg = "redone";
    return true;
  }
  msg = "unknown command: " + command;
  return false;
}

}  // namespace g1_map_manager
