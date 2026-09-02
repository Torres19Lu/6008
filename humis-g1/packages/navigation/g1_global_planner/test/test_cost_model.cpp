// CostModel unit tests: blocked thresholds, stepCost monotonicity.

#include <gtest/gtest.h>

#include "g1_global_planner/core/cost_model.h"
#include "g1_global_planner/core/plan_types.h"

using g1_global_planner::CostModel;
using g1_global_planner::PlannerConfig;

TEST(CostModel, BlockedLethal) {
  PlannerConfig cfg;  // lethal_threshold = 99 (blocks inscribed 99 + lethal 100)
  CostModel     cm(cfg);
  EXPECT_TRUE(cm.blocked(static_cast<int8_t>(100)));  // lethal
  EXPECT_TRUE(cm.blocked(static_cast<int8_t>(99)));   // inscribed, blocked for travel
}

TEST(CostModel, BlockedUnknownRespectsAllowUnknown) {
  PlannerConfig cfg;
  cfg.allow_unknown = false;
  CostModel cm_block(cfg);
  EXPECT_TRUE(cm_block.blocked(static_cast<int8_t>(-1)));

  cfg.allow_unknown = true;
  CostModel cm_allow(cfg);
  EXPECT_FALSE(cm_allow.blocked(static_cast<int8_t>(-1)));
}

TEST(CostModel, BlockedInflationNotBlocked) {
  PlannerConfig cfg;
  CostModel     cm(cfg);
  EXPECT_FALSE(cm.blocked(static_cast<int8_t>(50)));
  EXPECT_FALSE(cm.blocked(static_cast<int8_t>(1)));
  EXPECT_FALSE(cm.blocked(static_cast<int8_t>(98)));  // top of inflation band, traversable
}

TEST(CostModel, CustomLethalThreshold) {
  PlannerConfig cfg;
  cfg.lethal_threshold = 80;
  CostModel cm(cfg);
  EXPECT_TRUE(cm.blocked(static_cast<int8_t>(80)));
  EXPECT_FALSE(cm.blocked(static_cast<int8_t>(79)));
}

TEST(CostModel, StepCostFreeCell) {
  PlannerConfig cfg;
  cfg.inflation_cost_weight = 3.0;
  CostModel cm(cfg);
  EXPECT_DOUBLE_EQ(cm.stepCost(static_cast<int8_t>(0)), 1.0);
}

TEST(CostModel, StepCostUnknownWhenAllowed) {
  PlannerConfig cfg;
  cfg.allow_unknown = true;
  CostModel cm(cfg);
  EXPECT_DOUBLE_EQ(cm.stepCost(static_cast<int8_t>(-1)), 1.0);
}

TEST(CostModel, StepCostStrictlyIncreasing) {
  PlannerConfig cfg;
  cfg.inflation_cost_weight = 3.0;
  CostModel cm(cfg);
  for (int8_t v = 0; v < 99; ++v) {
    EXPECT_LT(cm.stepCost(v), cm.stepCost(static_cast<int8_t>(v + 1)))
        << "stepCost not strictly increasing at v=" << static_cast<int>(v);
  }
}

TEST(CostModel, StepCostHigherAtHigherInflation) {
  PlannerConfig cfg;
  cfg.inflation_cost_weight = 3.0;
  CostModel cm(cfg);
  EXPECT_GT(cm.stepCost(static_cast<int8_t>(99)),
            cm.stepCost(static_cast<int8_t>(1)));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
