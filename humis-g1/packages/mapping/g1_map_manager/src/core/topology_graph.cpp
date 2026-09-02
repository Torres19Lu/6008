// topology_graph.cpp -- Dijkstra routing over manual gateways.

#include "g1_map_manager/core/topology_graph.h"

#include <algorithm>
#include <functional>
#include <queue>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace g1_map_manager {

namespace {

Pose3 poseFromNode(const YAML::Node& n) {
  Pose3 p;
  if (n["position"]) {
    p.x = n["position"]["x"].as<double>();
    p.y = n["position"]["y"].as<double>();
    p.z = n["position"]["z"].as<double>();
  }
  if (n["orientation"]) {
    p.qx = n["orientation"]["x"].as<double>();
    p.qy = n["orientation"]["y"].as<double>();
    p.qz = n["orientation"]["z"].as<double>();
    p.qw = n["orientation"]["w"].as<double>();
  }
  return p;
}

void emitPose(YAML::Emitter& em, const char* key, const Eigen::Isometry3d& T) {
  const Pose3 p = fromIso(T);
  em << YAML::Key << key << YAML::Value << YAML::BeginMap;
  em << YAML::Key << "position" << YAML::Value << YAML::Flow << YAML::BeginMap
     << YAML::Key << "x" << YAML::Value << p.x << YAML::Key << "y" << YAML::Value
     << p.y << YAML::Key << "z" << YAML::Value << p.z << YAML::EndMap;
  em << YAML::Key << "orientation" << YAML::Value << YAML::Flow << YAML::BeginMap
     << YAML::Key << "x" << YAML::Value << p.qx << YAML::Key << "y" << YAML::Value
     << p.qy << YAML::Key << "z" << YAML::Value << p.qz << YAML::Key << "w"
     << YAML::Value << p.qw << YAML::EndMap;
  em << YAML::EndMap;
}

}  // namespace

void TopologyGraph::load(const std::vector<std::string>& maps,
                         const std::vector<GatewayEdge>& edges) {
  maps_ = maps;
  edges_ = edges;
  adj_.clear();
  for (const GatewayEdge& e : edges) {
    // Bidirectional: store the directional depart/seed for each traversal.
    adj_[e.from].push_back({e.to, e.pose_in_from, e.pose_in_to});
    adj_[e.to].push_back({e.from, e.pose_in_to, e.pose_in_from});
  }
  for (auto& kv : adj_) {
    std::sort(kv.second.begin(), kv.second.end(),
              [](const Adj& a, const Adj& b) { return a.next_map < b.next_map; });
  }
}

std::vector<RouteLeg> TopologyGraph::route(const std::string& from,
                                           const std::string& to) const {
  if (from == to) {
    RouteLeg leg;
    leg.map = from;
    leg.is_final = true;
    return {leg};
  }

  // Dijkstra with unit edge weights (minimize the number of map switches).
  std::map<std::string, double> dist;
  std::map<std::string, std::pair<std::string, Adj>> prev;  // node -> (pred, adj used)
  using QE = std::pair<double, std::string>;
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  dist[from] = 0.0;
  pq.push({0.0, from});
  while (!pq.empty()) {
    const double d = pq.top().first;
    const std::string u = pq.top().second;
    pq.pop();
    if (d > dist[u]) continue;
    if (u == to) break;
    const auto it = adj_.find(u);
    if (it == adj_.end()) continue;
    for (const Adj& a : it->second) {
      const double nd = d + 1.0;
      const auto di = dist.find(a.next_map);
      if (di == dist.end() || nd < di->second) {
        dist[a.next_map] = nd;
        prev[a.next_map] = {u, a};
        pq.push({nd, a.next_map});
      }
    }
  }
  if (dist.find(to) == dist.end()) return {};  // unreachable

  // Reconstruct the path edges (to -> from), then reverse.
  std::vector<std::pair<std::string, Adj>> steps;
  std::string cur = to;
  while (cur != from) {
    const auto& p = prev.at(cur);
    steps.push_back(p);
    cur = p.first;
  }
  std::reverse(steps.begin(), steps.end());

  std::vector<RouteLeg> legs;
  legs.reserve(steps.size() + 1);
  for (const auto& step : steps) {
    RouteLeg leg;
    leg.map = step.first;            // depart from this map
    leg.is_final = false;
    leg.depart_pose = step.second.depart;
    leg.next_map = step.second.next_map;
    leg.seed_pose = step.second.seed;
    legs.push_back(leg);
  }
  RouteLeg final_leg;
  final_leg.map = to;
  final_leg.is_final = true;
  legs.push_back(final_leg);
  return legs;
}

std::vector<GatewayEdge> TopologyGraph::neighbors(const std::string& map) const {
  std::vector<GatewayEdge> out;
  for (const GatewayEdge& e : edges_) {
    if (e.from == map || e.to == map) out.push_back(e);
  }
  return out;
}

bool parseTopology(const std::string& yaml, std::vector<std::string>& maps,
                   std::vector<GatewayEdge>& edges) {
  maps.clear();
  edges.clear();
  if (yaml.empty()) return true;
  const YAML::Node n = YAML::Load(yaml);
  if (n["maps"]) {
    for (const auto& m : n["maps"]) maps.push_back(m.as<std::string>());
  }
  if (n["gateways"]) {
    for (const auto& g : n["gateways"]) {
      GatewayEdge e;
      e.from = g["from_map"].as<std::string>();
      e.to = g["to_map"].as<std::string>();
      e.pose_in_from = toIso(poseFromNode(g["pose_in_from"]));
      e.pose_in_to = toIso(poseFromNode(g["pose_in_to"]));
      e.label = g["label"] ? g["label"].as<std::string>() : "";
      e.source = g["source"] ? g["source"].as<std::string>() : "manual";
      edges.push_back(e);
    }
  }
  return true;
}

std::string serializeTopology(const std::vector<std::string>& maps,
                              const std::vector<GatewayEdge>& edges) {
  YAML::Emitter em;
  em << YAML::BeginMap;
  em << YAML::Key << "version" << YAML::Value << 1;
  em << YAML::Key << "maps" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (const std::string& m : maps) em << m;
  em << YAML::EndSeq;
  em << YAML::Key << "gateways" << YAML::Value << YAML::BeginSeq;
  for (const GatewayEdge& e : edges) {
    em << YAML::BeginMap;
    em << YAML::Key << "from_map" << YAML::Value << e.from;
    em << YAML::Key << "to_map" << YAML::Value << e.to;
    emitPose(em, "pose_in_from", e.pose_in_from);
    emitPose(em, "pose_in_to", e.pose_in_to);
    em << YAML::Key << "label" << YAML::Value << e.label;
    em << YAML::Key << "source" << YAML::Value
       << (e.source.empty() ? "manual" : e.source);
    em << YAML::EndMap;
  }
  em << YAML::EndSeq;
  em << YAML::EndMap;
  return std::string(em.c_str()) + "\n";
}

}  // namespace g1_map_manager
