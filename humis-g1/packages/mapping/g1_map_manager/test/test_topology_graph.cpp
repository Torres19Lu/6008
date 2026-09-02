// TopologyGraph routing tests: same-map, single-hop, multi-hop shortest path,
// no-route, directional depart/seed poses, and neighbor derivation.

#include <gtest/gtest.h>

#include <vector>

#include <Eigen/Geometry>

#include "g1_map_manager/core/topology_graph.h"

using namespace g1_map_manager;

namespace {

Eigen::Isometry3d tx(double x) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << x, 0.0, 0.0;
  return T;
}

// Gateway with pose_in_from = tx(fx), pose_in_to = tx(tx_).
GatewayEdge edge(const std::string& from, const std::string& to, double fx,
                 double tx_) {
  GatewayEdge e;
  e.from = from;
  e.to = to;
  e.pose_in_from = tx(fx);
  e.pose_in_to = tx(tx_);
  e.source = "manual";
  return e;
}

double X(const Eigen::Isometry3d& T) { return T.translation().x(); }

}  // namespace

TEST(TopologyGraph, SameMapIsOneFinalLeg) {
  TopologyGraph g;
  g.load({"A", "B"}, {edge("A", "B", 1, 2)});
  const auto legs = g.route("A", "A");
  ASSERT_EQ(legs.size(), 1u);
  EXPECT_TRUE(legs[0].is_final);
  EXPECT_EQ(legs[0].map, "A");
}

TEST(TopologyGraph, SingleHopHasCrossingThenFinal) {
  TopologyGraph g;
  g.load({"A", "B"}, {edge("A", "B", 1, 2)});
  const auto legs = g.route("A", "B");
  ASSERT_EQ(legs.size(), 2u);
  EXPECT_FALSE(legs[0].is_final);
  EXPECT_EQ(legs[0].map, "A");
  EXPECT_EQ(legs[0].next_map, "B");
  EXPECT_DOUBLE_EQ(X(legs[0].depart_pose), 1.0);  // pose_in_from (A frame)
  EXPECT_DOUBLE_EQ(X(legs[0].seed_pose), 2.0);    // pose_in_to (B frame)
  EXPECT_TRUE(legs[1].is_final);
  EXPECT_EQ(legs[1].map, "B");
}

TEST(TopologyGraph, ReverseDirectionSwapsDepartAndSeed) {
  TopologyGraph g;
  g.load({"A", "B"}, {edge("A", "B", 1, 2)});
  const auto legs = g.route("B", "A");
  ASSERT_EQ(legs.size(), 2u);
  EXPECT_EQ(legs[0].map, "B");
  EXPECT_EQ(legs[0].next_map, "A");
  EXPECT_DOUBLE_EQ(X(legs[0].depart_pose), 2.0);  // pose_in_to (B frame)
  EXPECT_DOUBLE_EQ(X(legs[0].seed_pose), 1.0);    // pose_in_from (A frame)
  EXPECT_TRUE(legs[1].is_final);
  EXPECT_EQ(legs[1].map, "A");
}

TEST(TopologyGraph, MultiHopChainsLegs) {
  TopologyGraph g;
  g.load({"A", "B", "C"}, {edge("A", "B", 1, 2), edge("B", "C", 3, 4)});
  const auto legs = g.route("A", "C");
  ASSERT_EQ(legs.size(), 3u);
  EXPECT_EQ(legs[0].map, "A");
  EXPECT_EQ(legs[0].next_map, "B");
  EXPECT_EQ(legs[1].map, "B");
  EXPECT_EQ(legs[1].next_map, "C");
  EXPECT_DOUBLE_EQ(X(legs[1].depart_pose), 3.0);
  EXPECT_DOUBLE_EQ(X(legs[1].seed_pose), 4.0);
  EXPECT_TRUE(legs[2].is_final);
  EXPECT_EQ(legs[2].map, "C");
}

TEST(TopologyGraph, PrefersShortestPath) {
  TopologyGraph g;
  g.load({"A", "B", "C"},
         {edge("A", "B", 1, 2), edge("B", "C", 3, 4), edge("A", "C", 5, 6)});
  const auto legs = g.route("A", "C");
  ASSERT_EQ(legs.size(), 2u);  // direct A->C, not A->B->C
  EXPECT_EQ(legs[0].map, "A");
  EXPECT_EQ(legs[0].next_map, "C");
  EXPECT_DOUBLE_EQ(X(legs[0].depart_pose), 5.0);
}

TEST(TopologyGraph, NoRouteIsEmpty) {
  TopologyGraph g;
  g.load({"A", "B", "D"}, {edge("A", "B", 1, 2)});
  EXPECT_TRUE(g.route("A", "D").empty());
}

TEST(TopologyGraph, NeighborsAreIncidentEdges) {
  TopologyGraph g;
  g.load({"A", "B", "C"}, {edge("A", "B", 1, 2), edge("B", "C", 3, 4)});
  EXPECT_EQ(g.neighbors("B").size(), 2u);
  EXPECT_EQ(g.neighbors("A").size(), 1u);
}

TEST(TopologyGraph, TopologyYamlRoundTrip) {
  std::vector<std::string> maps = {"A", "B", "C"};
  std::vector<GatewayEdge> edges = {edge("A", "B", 1, 2), edge("B", "C", 3, 4)};
  edges[0].label = "door1";
  const std::string yaml = serializeTopology(maps, edges);

  std::vector<std::string> m2;
  std::vector<GatewayEdge> e2;
  ASSERT_TRUE(parseTopology(yaml, m2, e2));
  EXPECT_EQ(m2, maps);
  ASSERT_EQ(e2.size(), 2u);
  EXPECT_EQ(e2[0].from, "A");
  EXPECT_EQ(e2[0].to, "B");
  EXPECT_DOUBLE_EQ(X(e2[0].pose_in_from), 1.0);
  EXPECT_DOUBLE_EQ(X(e2[0].pose_in_to), 2.0);
  EXPECT_EQ(e2[0].label, "door1");
  EXPECT_EQ(e2[0].source, "manual");

  TopologyGraph g;
  g.load(m2, e2);
  EXPECT_EQ(g.route("A", "C").size(), 3u);  // parsed edges route correctly
}

TEST(TopologyGraph, ParseEmptyIsEmpty) {
  std::vector<std::string> m;
  std::vector<GatewayEdge> e;
  ASSERT_TRUE(parseTopology("", m, e));
  EXPECT_TRUE(m.empty());
  EXPECT_TRUE(e.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
