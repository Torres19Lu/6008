#pragma once
// progress_watchdog.h -- ProgressWatchdog: rolling-window progress detector.
// ROS-free, std::thread-free. Fires when net displacement over stuck_window
// is below stuck_dist. Never fires until the trail spans the full window.

#include "g1_nav/core/nav_types.h"
#include <deque>
#include <utility>

namespace g1_nav {

class ProgressWatchdog {
 public:
  explicit ProgressWatchdog(const WatchdogConfig& cfg);

  // Live reconfigure; takes effect on the next update()/noProgress() call.
  void setConfig(const WatchdogConfig& cfg);

  // Clear the pose trail. noProgress() returns false until the window refills.
  void reset();

  // Feed the latest pose and monotonic timestamp (seconds, same domain as now
  // in noProgress). Prunes entries older than stuck_window from the trail head.
  void update(const Pose2D& pose, double now);

  // Returns true iff the trail spans >= stuck_window seconds AND the net
  // first-to-last displacement is < stuck_dist. Safe to call without prior update.
  bool noProgress(double now) const;

 private:
  WatchdogConfig cfg_;

  // Chronological trail of (timestamp, pose).
  std::deque<std::pair<double, Pose2D>> trail_;
};

} // namespace g1_nav
