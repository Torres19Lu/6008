// progress_watchdog.cpp -- ProgressWatchdog implementation.
// ROS-free, std::thread-free.

#include "g1_nav/core/progress_watchdog.h"
#include <cmath>

namespace g1_nav {

ProgressWatchdog::ProgressWatchdog(const WatchdogConfig& cfg)
    : cfg_(cfg) {}

void ProgressWatchdog::setConfig(const WatchdogConfig& cfg) {
  cfg_ = cfg;
}

void ProgressWatchdog::reset() {
  trail_.clear();
}

void ProgressWatchdog::update(const Pose2D& pose, double now) {
  trail_.push_back({now, pose});

  // Prune entries that are older than stuck_window from the front.
  while (trail_.size() > 1 &&
         (now - trail_.front().first) > cfg_.stuck_window) {
    trail_.pop_front();
  }
}

bool ProgressWatchdog::noProgress(double now) const {
  // The window is measured from the stored trail timestamps; now is accepted for
  // API symmetry and future staleness checks (e.g. trail too old relative to now).
  (void)now;

  if (trail_.size() < 2) return false;

  // The window must be fully spanned before we can fire.
  double span = trail_.back().first - trail_.front().first;
  if (span < cfg_.stuck_window) return false;

  // Compute net displacement from the oldest to the newest entry.
  const Pose2D& first = trail_.front().second;
  const Pose2D& last  = trail_.back().second;
  double dx = last.x - first.x;
  double dy = last.y - first.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  return dist < cfg_.stuck_dist;
}

} // namespace g1_nav
