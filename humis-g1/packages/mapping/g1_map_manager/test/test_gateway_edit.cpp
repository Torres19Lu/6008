// GatewayEditor tests: define a gateway, nudge/yaw the seed, undo/redo, save +
// serialize round-trip, and that committing only touches the topology.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "g1_map_manager/core/gateway_editor.h"
#include "g1_map_manager/core/topology_graph.h"

using namespace g1_map_manager;

namespace {
GatewayEditArgs defineArgs() {
  GatewayEditArgs a;
  a.from_map = "m1";
  a.to_map = "m2";
  a.label = "door";
  a.pose_in_from.translation() << 3.0, 0.0, 0.0;
  a.pose_in_to.translation() << 7.0, 0.0, 0.0;
  return a;
}
}  // namespace

TEST(GatewayEditor, DefineNudgeYawStages) {
  GatewayEditor ed;
  ed.load({}, {});
  std::string msg;
  ASSERT_TRUE(ed.apply("define", defineArgs(), msg)) << msg;
  ASSERT_EQ(ed.view().size(), 1u);
  EXPECT_DOUBLE_EQ(ed.view()[0].pose_in_to.translation().x(), 7.0);

  GatewayEditArgs nudge;
  nudge.values = {0.5, -0.25};  // dx, dy on the seed
  ASSERT_TRUE(ed.apply("nudge", nudge, msg)) << msg;
  EXPECT_DOUBLE_EQ(ed.view()[0].pose_in_to.translation().x(), 7.5);
  EXPECT_DOUBLE_EQ(ed.view()[0].pose_in_to.translation().y(), -0.25);

  GatewayEditArgs yaw;
  yaw.values = {1.0};  // dyaw
  ASSERT_TRUE(ed.apply("yaw", yaw, msg)) << msg;
  const double got =
      Eigen::AngleAxisd(ed.view()[0].pose_in_to.rotation()).angle();
  EXPECT_NEAR(got, 1.0, 1e-9);
}

TEST(GatewayEditor, SaveCommitsAndSerializes) {
  GatewayEditor ed;
  ed.load({}, {});
  std::string msg;
  ASSERT_TRUE(ed.apply("define", defineArgs(), msg));
  ASSERT_TRUE(ed.apply("save", {}, msg));
  ASSERT_EQ(ed.edges().size(), 1u);
  EXPECT_TRUE(ed.dirty());

  // maps auto-registered; serialize round-trips and routes.
  std::vector<std::string> m2;
  std::vector<GatewayEdge> e2;
  ASSERT_TRUE(parseTopology(ed.serialize(), m2, e2));
  ASSERT_EQ(e2.size(), 1u);
  EXPECT_EQ(e2[0].from, "m1");
  EXPECT_EQ(e2[0].to, "m2");
  EXPECT_EQ(e2[0].source, "captured");
  TopologyGraph g;
  g.load(m2, e2);
  EXPECT_EQ(g.route("m1", "m2").size(), 2u);
}

TEST(GatewayEditor, UndoRedo) {
  GatewayEditor ed;
  ed.load({}, {});
  std::string msg;
  ed.apply("define", defineArgs(), msg);
  ed.apply("save", {}, msg);
  ASSERT_EQ(ed.edges().size(), 1u);

  ASSERT_TRUE(ed.apply("undo", {}, msg));
  EXPECT_EQ(ed.edges().size(), 0u);
  ASSERT_TRUE(ed.apply("redo", {}, msg));
  EXPECT_EQ(ed.edges().size(), 1u);
}

TEST(GatewayEditor, DiscardDropsStaged) {
  GatewayEditor ed;
  ed.load({}, {});
  std::string msg;
  ed.apply("define", defineArgs(), msg);
  ASSERT_EQ(ed.view().size(), 1u);
  ASSERT_TRUE(ed.apply("discard", {}, msg));
  EXPECT_EQ(ed.view().size(), 0u);
  EXPECT_EQ(ed.edges().size(), 0u);
}

TEST(GatewayEditor, SaveReplacesExistingFromToEdge) {
  GatewayEditor ed;
  ed.load({}, {});
  std::string msg;
  ASSERT_TRUE(ed.apply("define", defineArgs(), msg));
  ASSERT_TRUE(ed.apply("save", {}, msg));
  ASSERT_EQ(ed.edges().size(), 1u);

  GatewayEditArgs again = defineArgs();   // same m1->m2, new seed
  again.pose_in_to.translation() << 9.0, 0.0, 0.0;
  ASSERT_TRUE(ed.apply("define", again, msg));
  ASSERT_TRUE(ed.apply("save", {}, msg));
  ASSERT_EQ(ed.edges().size(), 1u);       // replaced, not appended
  EXPECT_DOUBLE_EQ(ed.edges()[0].pose_in_to.translation().x(), 9.0);
}

TEST(GatewayEditor, RefineIsRemoved) {
  GatewayEditor ed;
  ed.load({}, {});
  std::string msg;
  ASSERT_TRUE(ed.apply("define", defineArgs(), msg));
  EXPECT_FALSE(ed.apply("refine", {}, msg));   // unknown command now
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
