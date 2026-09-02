#pragma once
// map_types.h -- shared types for the g1_map_manager core (version store, topology
// routing, overlay, nav-point anchoring, mode machine). ROS-free: in-memory poses
// are Eigen::Isometry3d (SE(3) via Eigen, not hand-rolled quaternion ops); Pose3 is
// ONLY the YAML/msg (de)serialization DTO. namespace g1_map_manager.

#include <cstdint>
#include <string>

#include <Eigen/Geometry>

namespace g1_map_manager {

// Mirrors g1_msgs/MapManagerState mode constants.
enum class Mode : uint8_t {
  IDLE = 0, MAPPING = 1, INCREMENTAL = 2, LOCALIZATION = 3, EDITING = 4
};

// YAML/msg (de)serialization DTO ONLY. In-memory math uses Eigen::Isometry3d.
struct Pose3 { double x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 1; };

struct NavPointRec {
  std::string name;
  uint32_t anchor_id = 0;
  Eigen::Isometry3d rel = Eigen::Isometry3d::Identity();  // T_keyframe_navpoint
  uint8_t source = 1;                                     // 1 = SOURCE_USER
};

struct GatewayEdge {
  std::string from, to;
  Eigen::Isometry3d pose_in_from = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_in_to = Eigen::Isometry3d::Identity();
  std::string label, source;
};

struct KeyframePoseRec {
  uint32_t id = 0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // map-frame optimized pose
};

// YAML/msg boundary conversions (core/pose_math.cpp). Composition/inverse use the
// Eigen::Isometry3d operators directly (a * b, .inverse()); no custom pose algebra.
Eigen::Isometry3d toIso(const Pose3&);
Pose3 fromIso(const Eigen::Isometry3d&);

}  // namespace g1_map_manager
