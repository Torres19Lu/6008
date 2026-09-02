#pragma once
// map_lineage.h -- the manager-owned geometry/lineage.yaml sidecar: version/workflow
// metadata the SLAM backend has no concept of. It sits next to the backend-owned
// manifest.yaml (which the manager NEVER writes) and travels into a snapshot via the
// same copyTree. created_at is supplied by the caller (the node) so the core stays
// time-free and deterministic. parseBackendNumKeyframes does a read-only flat key:value
// parse of the backend manifest (for display); it never serializes it.
#include <cstdint>
#include <string>

namespace g1_map_manager {

struct MapLineage {
  std::string name;
  std::string created_at;  // ISO-8601, supplied by the caller
  std::string label;       // short human label, e.g. "extend west wing"
  std::string reason;      // initial | incremental | edit | imported
  std::string parent;      // snapshot id this geometry derived from, or ""
};

std::string serializeLineage(const MapLineage& m);
MapLineage parseLineage(const std::string& yaml);

// Read-only: pull num_keyframes out of the backend's flat key:value manifest.yaml.
uint32_t parseBackendNumKeyframes(const std::string& manifest_text);

}  // namespace g1_map_manager
