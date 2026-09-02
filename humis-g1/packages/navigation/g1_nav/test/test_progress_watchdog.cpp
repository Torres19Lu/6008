// test_progress_watchdog.cpp -- gtest coverage of ProgressWatchdog.
// ROS-free; links only g1_nav_core.

#include <gtest/gtest.h>
#include "g1_nav/core/progress_watchdog.h"

using namespace g1_nav;

static WatchdogConfig defaultWdCfg() {
  WatchdogConfig c;
  c.stuck_window = 4.0;
  c.stuck_dist   = 0.10;
  return c;
}

static Pose2D makePose(double x, double y) {
  Pose2D p; p.x = x; p.y = y; p.yaw = 0.0;
  return p;
}

// ---------------------------------------------------------------------------
// 1. NeverFires_BeforeWindowFull
// Feed poses at t=0,1,2,3 (span = 3s, window = 4s). noProgress must be false
// because the window is not yet full.
// ---------------------------------------------------------------------------
TEST(ProgressWatchdog, NeverFires_BeforeWindowFull) {
  ProgressWatchdog wd(defaultWdCfg());

  // All at the same position (worst-case candidate for firing).
  Pose2D p = makePose(1.0, 2.0);
  wd.update(p, 0.0);
  wd.update(p, 1.0);
  wd.update(p, 2.0);
  wd.update(p, 3.0);

  // Span is 3.0s, window is 4.0s: must NOT fire.
  EXPECT_FALSE(wd.noProgress(3.0));
}

// ---------------------------------------------------------------------------
// 2. Fires_WhenStationary_WindowFull
// Feed poses at t=0,1,2,3,4, all at the same position (window=4s).
// noProgress at t=4 must be true.
// ---------------------------------------------------------------------------
TEST(ProgressWatchdog, Fires_WhenStationary_WindowFull) {
  ProgressWatchdog wd(defaultWdCfg());

  Pose2D p = makePose(5.0, 5.0);
  for (int i = 0; i <= 4; ++i) {
    wd.update(p, static_cast<double>(i));
  }

  EXPECT_TRUE(wd.noProgress(4.0));
}

// ---------------------------------------------------------------------------
// 3. NoFire_WhenMoving
// Feed poses at t=0..4, each separated by >= stuck_dist in cumulative distance.
// Net displacement from first to last exceeds stuck_dist, so noProgress = false.
// ---------------------------------------------------------------------------
TEST(ProgressWatchdog, NoFire_WhenMoving) {
  ProgressWatchdog wd(defaultWdCfg());

  // Each step moves 0.10m in x, so net displacement from t=0 to t=4 is 0.40m.
  for (int i = 0; i <= 4; ++i) {
    Pose2D p = makePose(i * 0.10, 0.0);
    wd.update(p, static_cast<double>(i));
  }

  EXPECT_FALSE(wd.noProgress(4.0));
}

// ---------------------------------------------------------------------------
// 4. Reset_ReArms
// Fire once, call reset(), then feed again. Must not fire until window re-fills.
// ---------------------------------------------------------------------------
TEST(ProgressWatchdog, Reset_ReArms) {
  ProgressWatchdog wd(defaultWdCfg());

  Pose2D p = makePose(0.0, 0.0);
  for (int i = 0; i <= 4; ++i) {
    wd.update(p, static_cast<double>(i));
  }
  ASSERT_TRUE(wd.noProgress(4.0));

  // After reset, a fresh window must be built before the watchdog can fire.
  wd.reset();
  EXPECT_FALSE(wd.noProgress(4.0));

  // Re-feed 3s of data (span < 4s): still must not fire.
  double base = 10.0;
  wd.update(p, base + 0.0);
  wd.update(p, base + 1.0);
  wd.update(p, base + 2.0);
  wd.update(p, base + 3.0);
  EXPECT_FALSE(wd.noProgress(base + 3.0));

  // One more update completes the window.
  wd.update(p, base + 4.0);
  EXPECT_TRUE(wd.noProgress(base + 4.0));
}

// ---------------------------------------------------------------------------
// 5. TrailPruning_Bounded
// Feed 1000 updates (no sleep, advancing time by 0.1s each). Verify that
// noProgress behaves correctly and the watchdog does not crash or accumulate
// an unbounded trail.
// ---------------------------------------------------------------------------
TEST(ProgressWatchdog, TrailPruning_Bounded) {
  WatchdogConfig cfg = defaultWdCfg();
  cfg.stuck_window = 4.0;
  cfg.stuck_dist   = 0.10;
  ProgressWatchdog wd(cfg);

  // Feed 1000 updates all at the same position.
  Pose2D p = makePose(3.0, 7.0);
  for (int i = 0; i < 1000; ++i) {
    double t = i * 0.1;  // t = 0, 0.1, 0.2, ..., 99.9
    wd.update(p, t);
  }

  // At t=99.9, window is full and robot has not moved: must fire.
  EXPECT_TRUE(wd.noProgress(99.9));

  // Feed 1000 updates with movement.
  wd.reset();
  for (int i = 0; i < 1000; ++i) {
    double t = i * 0.1;
    Pose2D moving = makePose(i * 0.01, 0.0);  // 0.01m per tick, net >> stuck_dist
    wd.update(moving, t);
  }
  // Net displacement from oldest-in-window to newest far exceeds stuck_dist.
  EXPECT_FALSE(wd.noProgress(99.9));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
