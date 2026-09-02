// gateway_editor.cpp -- staged cross-map gateway authoring.

#include "g1_map_manager/core/gateway_editor.h"

#include <algorithm>

#include "g1_map_manager/core/topology_graph.h"  // serializeTopology

namespace g1_map_manager {

namespace {
void ensureMap(std::vector<std::string>& maps, const std::string& name) {
  if (!name.empty() &&
      std::find(maps.begin(), maps.end(), name) == maps.end()) {
    maps.push_back(name);
  }
}
}  // namespace

void GatewayEditor::load(const std::vector<std::string>& maps,
                         const std::vector<GatewayEdge>& edges) {
  maps_ = maps;
  edges_ = edges;
  has_staged_ = false;
  undo_.clear();
  redo_.clear();
  dirty_ = false;
}

void GatewayEditor::pushUndo() {
  undo_.push_back(edges_);
  redo_.clear();
}

std::vector<GatewayEdge> GatewayEditor::view() const {
  std::vector<GatewayEdge> v = edges_;
  if (has_staged_) v.push_back(staged_);
  return v;
}

std::string GatewayEditor::serialize() const {
  return serializeTopology(maps_, edges_);
}

bool GatewayEditor::apply(const std::string& command, const GatewayEditArgs& args,
                          std::string& msg) {
  if (command == "define") {
    staged_ = GatewayEdge{};
    staged_.from = args.from_map;
    staged_.to = args.to_map;
    staged_.label = args.label;
    staged_.source = "captured";
    staged_.pose_in_from = args.pose_in_from;
    staged_.pose_in_to = args.pose_in_to;
    has_staged_ = true;
    msg = "staged gateway " + args.from_map + " -> " + args.to_map;
    return true;
  }
  if (command == "select") {
    for (const GatewayEdge& e : edges_) {
      if (e.from == args.from_map && e.to == args.to_map) {
        staged_ = e;
        has_staged_ = true;
        msg = "selected gateway for re-edit";
        return true;
      }
    }
    msg = "no such gateway to select";
    return false;
  }
  if (command == "nudge") {
    if (!has_staged_) { msg = "nothing staged"; return false; }
    const double dx = args.values.size() > 0 ? args.values[0] : 0.0;
    const double dy = args.values.size() > 1 ? args.values[1] : 0.0;
    staged_.pose_in_to.translation().x() += dx;
    staged_.pose_in_to.translation().y() += dy;
    msg = "nudged seed";
    return true;
  }
  if (command == "yaw") {
    if (!has_staged_) { msg = "nothing staged"; return false; }
    const double dyaw = args.values.empty() ? 0.0 : args.values[0];
    staged_.pose_in_to.rotate(
        Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()));
    msg = "rotated seed";
    return true;
  }
  if (command == "status") {
    msg = std::to_string(edges_.size()) + " committed, " +
          (has_staged_ ? "1 staged" : "0 staged");
    return true;
  }
  if (command == "discard") {
    has_staged_ = false;
    msg = "discarded staged gateway";
    return true;
  }
  if (command == "save") {
    if (!has_staged_) { msg = "nothing staged to save"; return false; }
    pushUndo();
    bool replaced = false;
    for (GatewayEdge& e : edges_) {
      if (e.from == staged_.from && e.to == staged_.to) {
        e = staged_;          // re-capture updates the existing edge in place
        replaced = true;
        break;
      }
    }
    if (!replaced) edges_.push_back(staged_);
    ensureMap(maps_, staged_.from);
    ensureMap(maps_, staged_.to);
    has_staged_ = false;
    dirty_ = true;
    msg = replaced ? "updated gateway" : "saved gateway";
    return true;
  }
  if (command == "undo") {
    if (undo_.empty()) { msg = "nothing to undo"; return false; }
    redo_.push_back(edges_);
    edges_ = undo_.back();
    undo_.pop_back();
    dirty_ = true;
    msg = "undone";
    return true;
  }
  if (command == "redo") {
    if (redo_.empty()) { msg = "nothing to redo"; return false; }
    undo_.push_back(edges_);
    edges_ = redo_.back();
    redo_.pop_back();
    dirty_ = true;
    msg = "redone";
    return true;
  }
  msg = "unknown command: " + command;
  return false;
}

}  // namespace g1_map_manager
