#pragma once
// Pure RelocResult -> g1_msgs/RelocQuality map. Node-adjacent (includes a g1_msgs
// generated header), so it lives outside g1_slam_backend_core to keep the core
// ROS-free.

#include <g1_msgs/RelocQuality.h>

#include "g1_slam_backend/backend.h"

namespace g1_slam_backend {

inline g1_msgs::RelocQuality toRelocQualityMsg(const Backend::RelocResult& r) {
  g1_msgs::RelocQuality q;
  q.inlier_ratio = r.inlier_ratio;
  q.fitness = r.fitness;
  q.sc_distance = r.sc_distance;
  q.match_id = r.match_id;
  q.accepted = r.found;
  return q;
}

}  // namespace g1_slam_backend
