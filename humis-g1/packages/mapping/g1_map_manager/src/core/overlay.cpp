// overlay.cpp -- parse/serialize the non-destructive edit overlay (yaml-cpp).

#include "g1_map_manager/core/overlay.h"

#include <yaml-cpp/yaml.h>

namespace g1_map_manager {

Overlay parseOverlay(const std::string& obstacle_yaml, const std::string& trim_yaml,
                     const std::string& metadata_yaml) {
  Overlay o;

  if (!obstacle_yaml.empty()) {
    const YAML::Node n = YAML::Load(obstacle_yaml);
    if (n["deleted_voxels"]) {
      for (const auto& v : n["deleted_voxels"]) {
        o.deleted_voxels.push_back(
            {v[0].as<double>(), v[1].as<double>(), v[2].as<double>()});
      }
    }
  }

  if (!trim_yaml.empty()) {
    const YAML::Node n = YAML::Load(trim_yaml);
    if (n["suppressed_keyframes"]) {
      for (const auto& k : n["suppressed_keyframes"]) {
        o.suppressed_keyframes.push_back(k.as<std::uint32_t>());
      }
    }
    if (n["crops"]) {
      for (const auto& poly : n["crops"]) {
        std::vector<std::array<double, 2>> p;
        for (const auto& pt : poly) {
          p.push_back({pt[0].as<double>(), pt[1].as<double>()});
        }
        o.crops.push_back(std::move(p));
      }
    }
  }

  if (!metadata_yaml.empty()) {
    const YAML::Node n = YAML::Load(metadata_yaml);
    for (const auto& kv : n) {
      const std::string key = kv.first.as<std::string>();
      const YAML::Node val = kv.second;
      if (val.IsScalar()) {
        o.metadata[key] = val.as<std::string>();
      } else {
        YAML::Emitter em;
        em << val;
        o.metadata[key] = em.c_str();
      }
    }
  }

  return o;
}

std::string serializeObstacle(const Overlay& o) {
  YAML::Emitter em;
  em << YAML::BeginMap;
  em << YAML::Key << "deleted_voxels" << YAML::Value << YAML::BeginSeq;
  for (const auto& v : o.deleted_voxels) {
    em << YAML::Flow << YAML::BeginSeq << v[0] << v[1] << v[2] << YAML::EndSeq;
  }
  em << YAML::EndSeq;
  em << YAML::EndMap;
  return std::string(em.c_str()) + "\n";
}

std::string serializeTrim(const Overlay& o) {
  YAML::Emitter em;
  em << YAML::BeginMap;
  em << YAML::Key << "suppressed_keyframes" << YAML::Value << YAML::Flow
     << YAML::BeginSeq;
  for (const std::uint32_t id : o.suppressed_keyframes) em << id;
  em << YAML::EndSeq;
  em << YAML::Key << "crops" << YAML::Value << YAML::BeginSeq;
  for (const auto& poly : o.crops) {
    em << YAML::BeginSeq;
    for (const auto& pt : poly) {
      em << YAML::Flow << YAML::BeginSeq << pt[0] << pt[1] << YAML::EndSeq;
    }
    em << YAML::EndSeq;
  }
  em << YAML::EndSeq;
  em << YAML::EndMap;
  return std::string(em.c_str()) + "\n";
}

std::string serializeMetadata(const Overlay& o) {
  YAML::Emitter em;
  em << YAML::BeginMap;
  for (const auto& kv : o.metadata) {
    em << YAML::Key << kv.first << YAML::Value << kv.second;
  }
  em << YAML::EndMap;
  return std::string(em.c_str()) + "\n";
}

bool insidePolygon(const std::array<double, 2>& p,
                   const std::vector<std::array<double, 2>>& poly) {
  const std::size_t n = poly.size();
  if (n < 3) return false;
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = poly[i][0], yi = poly[i][1];
    const double xj = poly[j][0], yj = poly[j][1];
    const bool crosses = ((yi > p[1]) != (yj > p[1])) &&
                         (p[0] < (xj - xi) * (p[1] - yi) / (yj - yi) + xi);
    if (crosses) inside = !inside;
  }
  return inside;
}

}  // namespace g1_map_manager
