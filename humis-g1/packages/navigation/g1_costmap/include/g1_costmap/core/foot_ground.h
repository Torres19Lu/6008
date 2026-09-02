#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include <Eigen/Geometry>

namespace g1_costmap {

// Kinematic floor estimate from the feet. `centres_in_foot` are the foot sole
// collision-sphere centres in the *_ankle_roll_link frame (identical for both
// feet); `sole_radius` is their radius. Each centre is projected through the
// resolved foot pose (the full TF, so foot tilt is exact, not just the origin z)
// and the minimum map-frame Z over all projected centres, minus the radius, is
// the lowest sole point = the stance foot's floor. The min over points subsumes
// lower-foot (stance) selection. The caller guarantees has_left || has_right.
inline double footGroundFromContacts(
    const Eigen::Isometry3d& Tl, bool has_left,
    const Eigen::Isometry3d& Tr, bool has_right,
    const std::vector<Eigen::Vector3d>& centres_in_foot, double sole_radius) {
  double g = std::numeric_limits<double>::infinity();
  if (has_left)
    for (const auto& c : centres_in_foot) g = std::min(g, (Tl * c).z());
  if (has_right)
    for (const auto& c : centres_in_foot) g = std::min(g, (Tr * c).z());
  return g - sole_radius;
}

}  // namespace g1_costmap
