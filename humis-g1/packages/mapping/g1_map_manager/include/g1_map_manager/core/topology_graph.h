#pragma once
// topology_graph.h -- the global cross-map graph: nodes are map names, edges are
// manual gateways. route() runs Dijkstra (unit edge weights) and emits RouteLegs
// whose depart/seed poses are already oriented for the traversal direction, so the
// cross-map node never re-derives gateway direction. ROS-free.

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "g1_map_manager/core/map_types.h"

namespace g1_map_manager {

struct RouteLeg {
  std::string map;                  // map this leg executes in (the current map)
  bool is_final = false;            // true => goal leg (the node uses the user goal)
  Eigen::Isometry3d depart_pose =   // gateway departure pose in `map` frame (non-final)
      Eigen::Isometry3d::Identity();
  std::string next_map;             // map entered after crossing (non-final)
  Eigen::Isometry3d seed_pose =     // relocalization seed in next_map frame (non-final)
      Eigen::Isometry3d::Identity();
};

class TopologyGraph {
 public:
  void load(const std::vector<std::string>& maps,
            const std::vector<GatewayEdge>& edges);
  // Route from current map to target map; empty vector => no route; same map => a
  // single final leg.
  std::vector<RouteLeg> route(const std::string& from, const std::string& to) const;
  std::vector<GatewayEdge> neighbors(const std::string& map) const;
  const std::vector<std::string>& maps() const { return maps_; }
  const std::vector<GatewayEdge>& edges() const { return edges_; }

 private:
  struct Adj {
    std::string next_map;
    Eigen::Isometry3d depart;  // departure pose in the FROM map frame
    Eigen::Isometry3d seed;    // relocalization seed in the next_map frame
  };
  std::vector<std::string> maps_;
  std::vector<GatewayEdge> edges_;
  std::map<std::string, std::vector<Adj>> adj_;
};

// YAML <-> (maps, edges) for the global topology.yaml (the single source of truth).
bool parseTopology(const std::string& yaml, std::vector<std::string>& maps,
                   std::vector<GatewayEdge>& edges);
std::string serializeTopology(const std::vector<std::string>& maps,
                              const std::vector<GatewayEdge>& edges);

}  // namespace g1_map_manager
