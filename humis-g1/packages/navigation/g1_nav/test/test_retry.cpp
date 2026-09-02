// test_retry.cpp -- gtest for decideRetryAction (RETRY decision).
// ROS-free; links only g1_nav_core.

#include <gtest/gtest.h>
#include "g1_nav/core/retry.h"

using g1_nav::RetryAction;
using g1_nav::decideRetryAction;

TEST(DecideRetry, HoldBelowInterval) {
  EXPECT_EQ(decideRetryAction(/*elapsed*/2.0, /*since*/0.4, /*budget*/120.0, /*interval*/1.0),
            RetryAction::HOLD);
}

TEST(DecideRetry, AttemptAtInterval) {
  EXPECT_EQ(decideRetryAction(2.0, 1.0, 120.0, 1.0), RetryAction::ATTEMPT);
  EXPECT_EQ(decideRetryAction(2.0, 1.5, 120.0, 1.0), RetryAction::ATTEMPT);
}

TEST(DecideRetry, TimeoutPastBudget) {
  EXPECT_EQ(decideRetryAction(120.1, 1.5, 120.0, 1.0), RetryAction::TIMEOUT);
}

TEST(DecideRetry, TimeoutDominatesAttempt) {
  // Past budget AND past interval -> TIMEOUT wins.
  EXPECT_EQ(decideRetryAction(200.0, 50.0, 120.0, 1.0), RetryAction::TIMEOUT);
}

TEST(DecideRetry, UnboundedBudgetNeverTimesOut) {
  EXPECT_EQ(decideRetryAction(1e6, 1.0, 0.0, 1.0), RetryAction::ATTEMPT);
  EXPECT_EQ(decideRetryAction(1e6, 0.1, 0.0, 1.0), RetryAction::HOLD);
}

TEST(DecideRetry, AtBudgetBoundaryNotYetTimedOut) {
  // elapsed == budget is NOT past it (strict >). Still ATTEMPT/HOLD.
  EXPECT_EQ(decideRetryAction(120.0, 2.0, 120.0, 1.0), RetryAction::ATTEMPT);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
