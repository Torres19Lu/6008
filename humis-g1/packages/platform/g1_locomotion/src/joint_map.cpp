#include "g1_locomotion/joint_map.hpp"

#include <cstddef>

namespace g1_locomotion {

JointStateData buildJointState(const std::array<double, kLowStateMotorCount>& q,
                               const std::vector<JointMapEntry>& map) {
  JointStateData out;
  out.names.reserve(map.size());
  out.positions.reserve(map.size());
  for (const auto& e : map) {
    if (e.sdk_index < 0 ||
        static_cast<std::size_t>(e.sdk_index) >= q.size()) {
      continue;
    }
    out.names.push_back(e.name);
    out.positions.push_back(q[static_cast<std::size_t>(e.sdk_index)]);
  }
  return out;
}

}  // namespace g1_locomotion
