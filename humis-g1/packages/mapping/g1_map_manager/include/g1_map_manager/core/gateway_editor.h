#pragma once
// gateway_editor.h -- staged authoring of cross-map gateways over the global
// topology. Captures each gateway as two endpoint poses (departure in from_map,
// arrival seed in to_map), supports nudge/yaw adjustment and undo/redo, and
// serializes to topology.yaml. ROS-free; the node adds TF capture + file I/O.

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "g1_map_manager/core/map_types.h"

namespace g1_map_manager {

struct GatewayEditArgs {
  std::string from_map, to_map, label;
  Eigen::Isometry3d pose_in_from = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_in_to = Eigen::Isometry3d::Identity();
  std::vector<double> values;  // command scalars (dx, dy, dyaw, ...)
};

class GatewayEditor {
 public:
  void load(const std::vector<std::string>& maps,
            const std::vector<GatewayEdge>& edges);

  // command in: define|select|nudge|yaw|refine|status|undo|redo|discard|save.
  // Returns false (with msg) on an invalid command/state.
  bool apply(const std::string& command, const GatewayEditArgs& args,
             std::string& msg);

  const std::vector<GatewayEdge>& edges() const { return edges_; }
  const std::vector<std::string>& maps() const { return maps_; }
  std::vector<GatewayEdge> view() const;  // committed edges + the staged one (if any)
  std::string serialize() const;          // topology.yaml of the committed edges
  bool dirty() const { return dirty_; }
  void clearDirty() { dirty_ = false; }

 private:
  void pushUndo();
  std::vector<std::string> maps_;
  std::vector<GatewayEdge> edges_;
  GatewayEdge staged_;
  bool has_staged_ = false;
  std::vector<std::vector<GatewayEdge>> undo_, redo_;
  bool dirty_ = false;
};

}  // namespace g1_map_manager
