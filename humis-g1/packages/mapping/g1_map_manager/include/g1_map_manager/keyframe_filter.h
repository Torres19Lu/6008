#pragma once
// keyframe_filter.h -- apply an edit Overlay to a KeyframeCloudArray: drop suppressed
// keyframes, drop points within delete_radius of a deleted voxel, and (when crops
// exist) drop points outside ALL keep-inside crop polygons. Node-adjacent (uses the
// g1_msgs cloud type + PCL), header-only so it is unit-testable without a node.

#include <cmath>
#include <set>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <g1_msgs/KeyframeCloud.h>
#include <g1_msgs/KeyframeCloudArray.h>

#include "g1_map_manager/core/overlay.h"

namespace g1_map_manager {

inline bool pointDeleted(float x, float y, float z, const Overlay& ov, double r2) {
  for (const auto& v : ov.deleted_voxels) {
    const double dx = x - v[0], dy = y - v[1], dz = z - v[2];
    if (dx * dx + dy * dy + dz * dz <= r2) return true;
  }
  return false;
}

// crops are keep-inside: a point outside ALL crop polygons is dropped (only when at
// least one crop exists).
inline bool outsideAllCrops(float x, float y, const Overlay& ov) {
  if (ov.crops.empty()) return false;
  for (const auto& poly : ov.crops) {
    if (insidePolygon({static_cast<double>(x), static_cast<double>(y)}, poly)) {
      return false;
    }
  }
  return true;
}

inline bool overlayEmpty(const Overlay& ov) {
  return ov.deleted_voxels.empty() && ov.suppressed_keyframes.empty() &&
         ov.crops.empty();
}

inline g1_msgs::KeyframeCloudArray filterKeyframes(
    const g1_msgs::KeyframeCloudArray& in, const Overlay& ov, double delete_radius) {
  if (overlayEmpty(ov)) return in;  // pass-through

  g1_msgs::KeyframeCloudArray out;
  out.header = in.header;
  const double r2 = delete_radius * delete_radius;
  const std::set<std::uint64_t> suppressed(ov.suppressed_keyframes.begin(),
                                           ov.suppressed_keyframes.end());
  for (const auto& kf : in.keyframes) {
    if (suppressed.count(kf.id)) continue;  // drop the whole keyframe
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(kf.cloud, cloud);
    pcl::PointCloud<pcl::PointXYZI> kept;
    kept.reserve(cloud.size());
    for (const auto& p : cloud) {
      if (pointDeleted(p.x, p.y, p.z, ov, r2)) continue;
      if (outsideAllCrops(p.x, p.y, ov)) continue;
      kept.push_back(p);
    }
    g1_msgs::KeyframeCloud okf = kf;
    pcl::toROSMsg(kept, okf.cloud);
    okf.cloud.header = kf.cloud.header;
    out.keyframes.push_back(okf);
  }
  return out;
}

}  // namespace g1_map_manager
