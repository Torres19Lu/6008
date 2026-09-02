// map_lineage.cpp -- manager-owned geometry/lineage.yaml IO + read-only backend
// manifest reader (num_keyframes only, flat key:value).

#include "g1_map_manager/core/map_lineage.h"

#include <sstream>

#include <yaml-cpp/yaml.h>

namespace g1_map_manager {

std::string serializeLineage(const MapLineage& m) {
  YAML::Emitter em;
  em << YAML::BeginMap;
  em << YAML::Key << "name" << YAML::Value << m.name;
  em << YAML::Key << "created_at" << YAML::Value << m.created_at;
  em << YAML::Key << "label" << YAML::Value << m.label;
  em << YAML::Key << "reason" << YAML::Value << m.reason;
  em << YAML::Key << "parent" << YAML::Value << m.parent;
  em << YAML::EndMap;
  return std::string(em.c_str()) + "\n";
}

MapLineage parseLineage(const std::string& yaml) {
  MapLineage m;
  if (yaml.empty()) return m;
  const YAML::Node n = YAML::Load(yaml);
  if (!n || !n.IsMap()) return m;
  if (n["name"]) m.name = n["name"].as<std::string>();
  if (n["created_at"]) m.created_at = n["created_at"].as<std::string>();
  if (n["label"]) m.label = n["label"].as<std::string>();
  if (n["reason"]) m.reason = n["reason"].as<std::string>();
  if (n["parent"]) m.parent = n["parent"].as<std::string>();
  return m;
}

uint32_t parseBackendNumKeyframes(const std::string& manifest_text) {
  // Flat "key: value" lines; mirror the backend's own simple parser. No yaml-cpp so a
  // future backend format tweak (comments, ordering) cannot make this throw.
  std::istringstream is(manifest_text);
  std::string line;
  while (std::getline(is, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string key = line.substr(0, colon);
    const auto k0 = key.find_first_not_of(" \t");
    const auto k1 = key.find_last_not_of(" \t");
    if (k0 == std::string::npos) continue;
    key = key.substr(k0, k1 - k0 + 1);
    if (key != "num_keyframes") continue;
    try {
      return static_cast<uint32_t>(std::stoul(line.substr(colon + 1)));
    } catch (...) {
      return 0;
    }
  }
  return 0;
}

}  // namespace g1_map_manager
