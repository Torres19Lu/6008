// pose_math.cpp -- Pose3 (YAML/msg DTO) <-> Eigen::Isometry3d conversions.

#include "g1_map_manager/core/map_types.h"

namespace g1_map_manager {

Eigen::Isometry3d toIso(const Pose3& p) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
  q.normalize();
  T.linear() = q.toRotationMatrix();
  return T;
}

Pose3 fromIso(const Eigen::Isometry3d& T) {
  Pose3 p;
  const Eigen::Vector3d t = T.translation();
  p.x = t.x();
  p.y = t.y();
  p.z = t.z();
  Eigen::Quaterniond q(T.rotation());
  q.normalize();
  p.qx = q.x();
  p.qy = q.y();
  p.qz = q.z();
  p.qw = q.w();
  return p;
}

}  // namespace g1_map_manager
