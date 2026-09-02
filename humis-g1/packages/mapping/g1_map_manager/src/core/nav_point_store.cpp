// nav_point_store.cpp -- keyframe-anchored nav points + YAML I/O.

#include "g1_map_manager/core/nav_point_store.h"

#include <algorithm>
#include <limits>

#include <yaml-cpp/yaml.h>

namespace g1_map_manager {

namespace {
constexpr uint8_t kSourceObject = 0;
constexpr uint8_t kSourceUser = 1;

const KeyframePoseRec* findKf(const std::vector<KeyframePoseRec>& kfs, uint32_t id) {
  for (const KeyframePoseRec& k : kfs) {
    if (k.id == id) return &k;
  }
  return nullptr;
}
}  // namespace

void NavPointStore::load(const std::string& nav_points_yaml) {
  points_.clear();
  map_name_.clear();
  undo_.clear();
  redo_.clear();
  if (nav_points_yaml.empty()) return;
  const YAML::Node n = YAML::Load(nav_points_yaml);
  if (n["map_name"]) map_name_ = n["map_name"].as<std::string>();
  if (!n["points"]) return;
  for (const auto& p : n["points"]) {
    NavPointRec rec;
    rec.name = p["name"].as<std::string>();
    rec.anchor_id = p["anchor_keyframe_id"].as<uint32_t>();
    Pose3 dto;
    const YAML::Node ar = p["anchor_relative_pose"];
    dto.x = ar["position"]["x"].as<double>();
    dto.y = ar["position"]["y"].as<double>();
    dto.z = ar["position"]["z"].as<double>();
    dto.qx = ar["orientation"]["x"].as<double>();
    dto.qy = ar["orientation"]["y"].as<double>();
    dto.qz = ar["orientation"]["z"].as<double>();
    dto.qw = ar["orientation"]["w"].as<double>();
    rec.rel = toIso(dto);
    rec.source = (p["source"] && p["source"].as<std::string>() == "object")
                     ? kSourceObject
                     : kSourceUser;
    points_.push_back(rec);
  }
}

std::string NavPointStore::serialize() const {
  YAML::Emitter em;
  em << YAML::BeginMap;
  em << YAML::Key << "map_name" << YAML::Value << map_name_;
  em << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;
  for (const NavPointRec& rec : points_) {
    const Pose3 dto = fromIso(rec.rel);
    em << YAML::BeginMap;
    em << YAML::Key << "name" << YAML::Value << rec.name;
    em << YAML::Key << "anchor_keyframe_id" << YAML::Value << rec.anchor_id;
    em << YAML::Key << "anchor_relative_pose" << YAML::Value << YAML::BeginMap;
    em << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginMap
       << YAML::Key << "x" << YAML::Value << dto.x << YAML::Key << "y" << YAML::Value
       << dto.y << YAML::Key << "z" << YAML::Value << dto.z << YAML::EndMap;
    em << YAML::Key << "orientation" << YAML::Value << YAML::Flow << YAML::BeginMap
       << YAML::Key << "x" << YAML::Value << dto.qx << YAML::Key << "y" << YAML::Value
       << dto.qy << YAML::Key << "z" << YAML::Value << dto.qz << YAML::Key << "w"
       << YAML::Value << dto.qw << YAML::EndMap;
    em << YAML::EndMap;
    em << YAML::Key << "source" << YAML::Value
       << (rec.source == kSourceObject ? "object" : "user");
    em << YAML::EndMap;
  }
  em << YAML::EndSeq;
  em << YAML::EndMap;
  return std::string(em.c_str()) + "\n";
}

bool NavPointStore::has(const std::string& name) const {
  return std::any_of(points_.begin(), points_.end(),
                     [&](const NavPointRec& r) { return r.name == name; });
}

bool NavPointStore::add(const std::string& name, const Pose3& world,
                        const std::vector<KeyframePoseRec>& kfs, double max_dist,
                        bool overwrite, NavPointRec& out, std::string& err) {
  if (has(name) && !overwrite) {
    err = "nav point '" + name + "' already exists (use overwrite)";
    return false;
  }
  if (kfs.empty()) {
    err = "no keyframes available to anchor against";
    return false;
  }
  const Eigen::Isometry3d world_iso = toIso(world);
  const Eigen::Vector3d wt = world_iso.translation();
  const KeyframePoseRec* nearest = nullptr;
  double best = std::numeric_limits<double>::max();
  for (const KeyframePoseRec& k : kfs) {
    const double d = (k.pose.translation() - wt).norm();
    if (d < best) {
      best = d;
      nearest = &k;
    }
  }
  if (best > max_dist) {
    err = "no keyframe within " + std::to_string(max_dist) + " m (nearest " +
          std::to_string(best) + " m)";
    return false;
  }

  pushUndo();
  NavPointRec rec;
  rec.name = name;
  rec.anchor_id = nearest->id;
  rec.rel = nearest->pose.inverse() * world_iso;  // T_keyframe_navpoint
  rec.source = kSourceUser;

  points_.erase(std::remove_if(points_.begin(), points_.end(),
                               [&](const NavPointRec& r) { return r.name == name; }),
                points_.end());
  points_.push_back(rec);
  out = rec;
  return true;
}

bool NavPointStore::resolve(const std::string& name,
                            const std::vector<KeyframePoseRec>& kfs,
                            Pose3& world_out) const {
  for (const NavPointRec& rec : points_) {
    if (rec.name != name) continue;
    const KeyframePoseRec* anchor = findKf(kfs, rec.anchor_id);
    if (!anchor) return false;  // anchor keyframe missing (e.g. trimmed)
    world_out = fromIso(anchor->pose * rec.rel);
    return true;
  }
  return false;
}

bool NavPointStore::remove(const std::string& name) {
  if (!has(name)) return false;
  pushUndo();
  const auto before = points_.size();
  points_.erase(std::remove_if(points_.begin(), points_.end(),
                               [&](const NavPointRec& r) { return r.name == name; }),
                points_.end());
  return points_.size() != before;
}

void NavPointStore::pushUndo() {
  undo_.push_back(points_);
  redo_.clear();
}

bool NavPointStore::undo() {
  if (undo_.empty()) return false;
  redo_.push_back(points_);
  points_ = undo_.back();
  undo_.pop_back();
  return true;
}

bool NavPointStore::redo() {
  if (redo_.empty()) return false;
  undo_.push_back(points_);
  points_ = redo_.back();
  redo_.pop_back();
  return true;
}

}  // namespace g1_map_manager
