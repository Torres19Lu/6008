#pragma once
// retry.h -- ROS-free retry decision for the RETRY state.
// Budget is by TIME, not count. TIMEOUT dominates ATTEMPT. budget <= 0 = unbounded.

#include <cstdint>

namespace g1_nav {

enum class RetryAction : uint8_t { HOLD = 0, ATTEMPT = 1, TIMEOUT = 2 };

// elapsed_sec            : seconds since entering RETRY (budget clock)
// since_last_attempt_sec : seconds since the last replan attempt
// budget_sec             : total retry budget; <= 0 means unbounded
// interval_sec           : minimum spacing between replan attempts
RetryAction decideRetryAction(double elapsed_sec, double since_last_attempt_sec,
                              double budget_sec, double interval_sec);

}  // namespace g1_nav
