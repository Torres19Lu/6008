#include "g1_nav/core/retry.h"

namespace g1_nav {

RetryAction decideRetryAction(double elapsed_sec, double since_last_attempt_sec,
                              double budget_sec, double interval_sec) {
  if (budget_sec > 0.0 && elapsed_sec > budget_sec) {
    return RetryAction::TIMEOUT;
  }
  if (since_last_attempt_sec >= interval_sec) {
    return RetryAction::ATTEMPT;
  }
  return RetryAction::HOLD;
}

}  // namespace g1_nav
