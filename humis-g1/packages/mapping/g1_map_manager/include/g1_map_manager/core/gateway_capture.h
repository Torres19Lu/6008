#pragma once
// gateway_capture.h -- ROS-free capture FSM for automatic gateway authoring. Enforces
// begin -> load -> commit ordering and holds the pending from-side capture. On a
// successful commit it assembles the GatewayEdge and returns to IDLE, handing the edge
// to the GatewayEditor for review/nudge/save. No ROS, no I/O.

#include <cstdint>
#include <string>

#include <Eigen/Geometry>

#include "g1_map_manager/core/map_types.h"

namespace g1_map_manager {

enum class CaptureState : std::uint8_t { IDLE = 0, BEGUN = 1, LOADED = 2 };

// ROS-free mirror of g1_msgs/RelocQuality (the node converts to/from the msg).
struct RelocQual {
  double inlier_ratio = 0.0;
  double fitness = 0.0;
  double sc_distance = 0.0;
  std::uint64_t match_id = 0;
  bool accepted = false;
};

class CaptureMachine {
 public:
  CaptureState state() const { return state_; }
  const std::string& fromMap() const { return from_map_; }
  const std::string& toMap() const { return to_map_; }
  const std::string& label() const { return label_; }
  const RelocQual& qualityFrom() const { return quality_from_; }

  // Record the from-side capture. Requires IDLE. to_map must be non-empty and != from.
  bool begin(const std::string& from_map, const std::string& to_map,
             const std::string& label, const Eigen::Isometry3d& pose_in_from,
             const RelocQual& quality_from, std::string& msg);

  // Mark the target loaded. Requires BEGUN.
  bool load(std::string& msg);

  // Record the to-side capture. Requires LOADED. If quality_to.accepted is false the
  // commit is rejected and the state stays LOADED (retry without reload). On success
  // assembles out_edge (source="captured") and returns to IDLE.
  bool commit(const Eigen::Isometry3d& pose_in_to, const RelocQual& quality_to,
              GatewayEdge& out_edge, std::string& msg);

  // Clear the pending capture. Requires a non-IDLE state.
  bool discard(std::string& msg);

 private:
  CaptureState state_ = CaptureState::IDLE;
  std::string from_map_, to_map_, label_;
  Eigen::Isometry3d pose_in_from_ = Eigen::Isometry3d::Identity();
  RelocQual quality_from_;
};

}  // namespace g1_map_manager
