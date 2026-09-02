// ModeMachine tests: write-gating by mode, gateway dwell, seed lock, global
// fallback, fallback timeout, and the transition flag.

#include <gtest/gtest.h>

#include <vector>

#include "g1_map_manager/core/mode_machine.h"
#include "g1_map_manager/core/topology_graph.h"

using namespace g1_map_manager;

namespace {

RouteLeg crossing() {
  RouteLeg l;
  l.is_final = false;
  l.map = "A";
  l.next_map = "B";
  return l;
}
RouteLeg finalLeg(const std::string& m) {
  RouteLeg l;
  l.is_final = true;
  l.map = m;
  return l;
}
ManagerObs at(double now) {
  ManagerObs o;
  o.now = now;
  return o;
}
ManagerObs legOk(double now) {
  ManagerObs o = at(now);
  o.leg_succeeded = true;
  return o;
}
ManagerObs locked(double now) {
  ManagerObs o = at(now);
  o.reloc_locked = true;
  return o;
}
ManagerObs failed(double now) {
  ManagerObs o = at(now);
  o.reloc_failed = true;
  return o;
}

}  // namespace

TEST(ModeMachine, CanWriteMapOnlyFalseInLocalization) {
  ModeMachine m;
  m.setMode(Mode::MAPPING);
  EXPECT_TRUE(m.canWriteMap());
  m.setMode(Mode::INCREMENTAL);
  EXPECT_TRUE(m.canWriteMap());
  m.setMode(Mode::EDITING);
  EXPECT_TRUE(m.canWriteMap());
  m.setMode(Mode::LOCALIZATION);
  EXPECT_FALSE(m.canWriteMap());
}

TEST(ModeMachine, TwoLegRouteDwellsThenSwitchesThenFinishes) {
  ModeMachine m;
  m.beginRoute({crossing(), finalLeg("B")}, 2.0, 0.0);

  XmDecision d = m.step(at(0.0));
  EXPECT_EQ(d.action, XmAction::SEND_LEG);
  EXPECT_EQ(d.leg_index, 0u);
  EXPECT_FALSE(d.transition);

  d = m.step(legOk(0.0));  // reached the gateway -> dwell begins
  EXPECT_EQ(d.action, XmAction::DWELL_GATEWAY);
  EXPECT_TRUE(d.transition);

  d = m.step(at(1.0));  // still dwelling
  EXPECT_EQ(d.action, XmAction::DWELL_GATEWAY);

  d = m.step(at(2.5));  // dwell elapsed -> switch
  EXPECT_EQ(d.action, XmAction::SWITCH_MAP);
  EXPECT_TRUE(d.transition);

  d = m.step(locked(2.5));  // seed locked -> next leg
  EXPECT_EQ(d.action, XmAction::SEND_LEG);
  EXPECT_EQ(d.leg_index, 1u);
  EXPECT_FALSE(d.transition);

  d = m.step(legOk(3.0));  // final leg done
  EXPECT_EQ(d.action, XmAction::FINISH_OK);
}

TEST(ModeMachine, ZeroDwellSwitchesImmediately) {
  ModeMachine m;
  m.beginRoute({crossing(), finalLeg("B")}, 0.0, 0.0);
  EXPECT_EQ(m.step(at(0.0)).action, XmAction::SEND_LEG);
  EXPECT_EQ(m.step(legOk(0.0)).action, XmAction::SWITCH_MAP);  // no dwell
}

TEST(ModeMachine, SeedFailureTriggersGlobalFallbackThenLock) {
  ModeMachine m;
  m.beginRoute({crossing(), finalLeg("B")}, 0.0, 0.0);
  m.step(at(0.0));            // SEND_LEG
  m.step(legOk(0.0));         // SWITCH_MAP (zero dwell)
  XmDecision d = m.step(failed(1.0));
  EXPECT_EQ(d.action, XmAction::GLOBAL_FALLBACK);
  EXPECT_TRUE(d.transition);
  d = m.step(locked(2.0));   // global acquire locks
  EXPECT_EQ(d.action, XmAction::SEND_LEG);
  EXPECT_EQ(d.leg_index, 1u);
}

TEST(ModeMachine, FallbackTimeoutFinishesFail) {
  ModeMachine m;
  m.setGlobalTimeout(5.0);
  m.beginRoute({crossing(), finalLeg("B")}, 0.0, 0.0);
  m.step(at(0.0));            // SEND_LEG
  m.step(legOk(0.0));         // SWITCH_MAP
  m.step(failed(1.0));        // -> GLOBAL_FALLBACK, fallback_start = 1.0
  XmDecision d = m.step(failed(6.5));  // 6.5 - 1.0 >= 5.0
  EXPECT_EQ(d.action, XmAction::FINISH_FAIL);
}

TEST(ModeMachine, LegAbortFinishesFail) {
  ModeMachine m;
  m.beginRoute({crossing(), finalLeg("B")}, 1.0, 0.0);
  m.step(at(0.0));  // SEND_LEG
  ManagerObs ab = at(0.5);
  ab.leg_aborted = true;
  EXPECT_EQ(m.step(ab).action, XmAction::FINISH_FAIL);
}

TEST(ModeMachine, SameMapRouteIsSingleLeg) {
  ModeMachine m;
  m.beginRoute({finalLeg("A")}, 0.0, 0.0);
  EXPECT_EQ(m.step(at(0.0)).action, XmAction::SEND_LEG);
  EXPECT_EQ(m.step(legOk(0.1)).action, XmAction::FINISH_OK);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
