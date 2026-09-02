// gateway_capture.cpp -- the capture FSM (see gateway_capture.h).

#include "g1_map_manager/core/gateway_capture.h"

namespace g1_map_manager {

bool CaptureMachine::begin(const std::string& from_map, const std::string& to_map,
                           const std::string& label,
                           const Eigen::Isometry3d& pose_in_from,
                           const RelocQual& quality_from, std::string& msg) {
  if (state_ != CaptureState::IDLE) {
    msg = "a capture is already pending; discard it first";
    return false;
  }
  if (from_map.empty() || to_map.empty()) {
    msg = "from_map and to_map must be set";
    return false;
  }
  if (from_map == to_map) {
    msg = "from_map and to_map must differ";
    return false;
  }
  from_map_ = from_map;
  to_map_ = to_map;
  label_ = label;
  pose_in_from_ = pose_in_from;
  quality_from_ = quality_from;
  state_ = CaptureState::BEGUN;
  msg = "began capture " + from_map_ + " -> " + to_map_;
  return true;
}

bool CaptureMachine::load(std::string& msg) {
  if (state_ != CaptureState::BEGUN) {
    msg = "load requires a pending begin";
    return false;
  }
  state_ = CaptureState::LOADED;
  msg = "target '" + to_map_ + "' marked loaded";
  return true;
}

bool CaptureMachine::commit(const Eigen::Isometry3d& pose_in_to,
                            const RelocQual& quality_to, GatewayEdge& out_edge,
                            std::string& msg) {
  if (state_ != CaptureState::LOADED) {
    msg = "commit requires a loaded target";
    return false;
  }
  if (!quality_to.accepted) {
    msg = "relocalization not accepted; stay loaded and retry";
    return false;  // keep LOADED for retry
  }
  out_edge = GatewayEdge{};
  out_edge.from = from_map_;
  out_edge.to = to_map_;
  out_edge.label = label_;
  out_edge.source = "captured";
  out_edge.pose_in_from = pose_in_from_;
  out_edge.pose_in_to = pose_in_to;
  state_ = CaptureState::IDLE;
  from_map_.clear();
  to_map_.clear();
  label_.clear();
  msg = "committed gateway";
  return true;
}

bool CaptureMachine::discard(std::string& msg) {
  if (state_ == CaptureState::IDLE) {
    msg = "no pending capture to discard";
    return false;
  }
  state_ = CaptureState::IDLE;
  from_map_.clear();
  to_map_.clear();
  label_.clear();
  msg = "discarded pending capture";
  return true;
}

}  // namespace g1_map_manager
