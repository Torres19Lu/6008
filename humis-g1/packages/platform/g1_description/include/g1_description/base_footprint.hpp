#ifndef G1_DESCRIPTION_BASE_FOOTPRINT_HPP
#define G1_DESCRIPTION_BASE_FOOTPRINT_HPP

#include <cmath>
#include <Eigen/Geometry>

namespace g1_description {

// Project base_link onto the ground plane: keep x/y, set z = 0, zero roll and
// pitch, keep heading (yaw of the base x-axis projected onto the ground).
// Input  : odom_T_base   (odom -> base_link)
// Returns: base_T_footprint (base_link -> base_footprint)
inline Eigen::Isometry3d computeBaseToFootprint(const Eigen::Isometry3d& odom_T_base) {
  // Heading = direction of the base x-axis projected onto the ground plane.
  const Eigen::Vector3d x_axis = odom_T_base.linear().col(0);
  const double yaw = std::atan2(x_axis.y(), x_axis.x());

  Eigen::Isometry3d odom_T_footprint = Eigen::Isometry3d::Identity();
  odom_T_footprint.translation() =
      Eigen::Vector3d(odom_T_base.translation().x(), odom_T_base.translation().y(), 0.0);
  odom_T_footprint.linear() =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  return odom_T_base.inverse() * odom_T_footprint;
}

}  // namespace g1_description

#endif  // G1_DESCRIPTION_BASE_FOOTPRINT_HPP
