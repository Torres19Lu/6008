#pragma once
// overlay.h -- the non-destructive edit overlay model: voxel/region deletions,
// keyframe trims (suppressed ids + keep-inside crop polygons), and metadata
// overrides. Parses/serializes the edits/*.yaml files. ROS-free (yaml-cpp only).

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace g1_map_manager {

struct Overlay {
  std::vector<std::array<double, 3>> deleted_voxels;          // obstacle_mask
  std::vector<std::uint32_t> suppressed_keyframes;            // keyframe_trim ids
  std::vector<std::vector<std::array<double, 2>>> crops;      // keep-inside polygons
  std::map<std::string, std::string> metadata;               // name/description/extent overrides
};

// Parse the three edit files (any may be ""). Missing keys yield empty members.
Overlay parseOverlay(const std::string& obstacle_yaml, const std::string& trim_yaml,
                     const std::string& metadata_yaml);

std::string serializeObstacle(const Overlay&);
std::string serializeTrim(const Overlay&);
std::string serializeMetadata(const Overlay&);

// Even-odd ray-cast point-in-polygon (xy). Polygons with < 3 vertices => false.
bool insidePolygon(const std::array<double, 2>& p,
                   const std::vector<std::array<double, 2>>& poly);

}  // namespace g1_map_manager
