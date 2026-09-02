#pragma once

#include <cstdint>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "g1_slam_backend/scan_context.h"

namespace g1_slam_backend {

// One pose-graph keyframe. The cloud is stored in this keyframe's base_link
// frame (storage 3: C_base = P_odom^-1 * cloud_world), so re-projecting it at the
// optimized pose reconstructs the map and the stored coordinates stay local.
struct Keyframe {
  std::uint64_t id = 0;
  double stamp = 0.0;
  Eigen::Isometry3d odom_pose = Eigen::Isometry3d::Identity();  // P_i = T(odom<-base_link)
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base;             // base_link_i frame
  ScanContextDB::Descriptor sc;                                // descriptor (SC frame)
};

}  // namespace g1_slam_backend
