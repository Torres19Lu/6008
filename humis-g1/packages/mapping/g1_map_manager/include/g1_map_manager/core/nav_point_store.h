#pragma once
// nav_point_store.h -- map-scoped, keyframe-anchored nav points. A point stores the
// nearest keyframe id + the relative transform T_keyframe_navpoint, so its world pose
// is resolved as keyframe_world_pose(anchor) * rel and tracks loop-closure
// re-optimization. ROS-free (Eigen + yaml-cpp via map_types).

#include <string>
#include <vector>

#include "g1_map_manager/core/map_types.h"

namespace g1_map_manager {

class NavPointStore {
 public:
  void load(const std::string& nav_points_yaml);  // "" => empty
  std::string serialize() const;

  void setMapName(const std::string& name) { map_name_ = name; }
  const std::string& mapName() const { return map_name_; }

  // Anchor `world` (map frame) to the nearest keyframe within `max_dist`; stores
  // rel = inverse(kf_pose) * world. Returns false (with err) if no keyframe is near
  // enough, or the name exists and !overwrite.
  bool add(const std::string& name, const Pose3& world,
           const std::vector<KeyframePoseRec>& kfs, double max_dist, bool overwrite,
           NavPointRec& out, std::string& err);

  // world = kf_pose(anchor) * rel; false if the name or its anchor keyframe is missing.
  bool resolve(const std::string& name, const std::vector<KeyframePoseRec>& kfs,
               Pose3& world_out) const;

  bool remove(const std::string& name);
  std::vector<NavPointRec> all() const { return points_; }
  bool has(const std::string& name) const;

  // In-session undo/redo over the points vector (mirrors MapEditor/GatewayEditor).
  // add()/remove() snapshot the pre-mutation state; load() clears both stacks.
  bool undo();
  bool redo();
  bool canUndo() const { return !undo_.empty(); }
  bool canRedo() const { return !redo_.empty(); }

 private:
  void pushUndo();

  std::string map_name_;
  std::vector<NavPointRec> points_;
  std::vector<std::vector<NavPointRec>> undo_, redo_;
};

}  // namespace g1_map_manager
