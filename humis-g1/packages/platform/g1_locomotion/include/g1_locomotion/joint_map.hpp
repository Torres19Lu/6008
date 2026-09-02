#ifndef G1_LOCOMOTION_JOINT_MAP_HPP
#define G1_LOCOMOTION_JOINT_MAP_HPP

#include <array>
#include <string>
#include <vector>

#include "g1_locomotion/loco_types.hpp"

namespace g1_locomotion {

// Parallel name/position vectors for a sensor_msgs/JointState.
struct JointStateData {
  std::vector<std::string> names;
  std::vector<double> positions;
};

// Translate a motor position array `q` (indexed 0..kLowStateMotorCount-1) into
// named joint positions using `map`. Entries with an out-of-range sdk_index are
// skipped. Output order matches `map` order.
JointStateData buildJointState(const std::array<double, kLowStateMotorCount>& q,
                               const std::vector<JointMapEntry>& map);

}  // namespace g1_locomotion
#endif  // G1_LOCOMOTION_JOINT_MAP_HPP
