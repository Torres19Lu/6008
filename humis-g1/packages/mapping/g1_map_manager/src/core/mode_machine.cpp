// mode_machine.cpp -- mode + cross-map transition state machine.

#include "g1_map_manager/core/mode_machine.h"

namespace g1_map_manager {

void ModeMachine::beginRoute(const std::vector<RouteLeg>& legs,
                             double transition_wait, double now) {
  legs_ = legs;
  transition_wait_ = transition_wait;
  route_start_ = now;
  leg_index_ = 0;
  phase_ = legs_.empty() ? Phase::FINISHED : Phase::SEND_LEG;
}

XmDecision ModeMachine::evalDwell(const ManagerObs& obs) {
  if (obs.now - dwell_start_ >= transition_wait_) {
    phase_ = Phase::AWAIT_SEED;
    return {XmAction::SWITCH_MAP, leg_index_, true};
  }
  return {XmAction::DWELL_GATEWAY, leg_index_, true};
}

XmDecision ModeMachine::step(const ManagerObs& obs) {
  switch (phase_) {
    case Phase::SEND_LEG:
      phase_ = Phase::AWAIT_LEG;
      return {XmAction::SEND_LEG, leg_index_, false};

    case Phase::AWAIT_LEG:
      if (obs.leg_aborted) {
        phase_ = Phase::FINISHED;
        return {XmAction::FINISH_FAIL, leg_index_, false};
      }
      if (obs.leg_succeeded) {
        if (legs_[leg_index_].is_final) {
          phase_ = Phase::FINISHED;
          return {XmAction::FINISH_OK, leg_index_, false};
        }
        // Crossing leg reached the gateway: begin the dwell (evaluated now so a
        // zero dwell switches immediately).
        dwell_start_ = obs.now;
        phase_ = Phase::DWELL;
        return evalDwell(obs);
      }
      return {XmAction::NONE, leg_index_, false};

    case Phase::DWELL:
      return evalDwell(obs);

    case Phase::AWAIT_SEED:
      if (obs.reloc_locked) {
        ++leg_index_;
        phase_ = Phase::AWAIT_LEG;
        return {XmAction::SEND_LEG, leg_index_, false};
      }
      if (obs.reloc_failed) {
        fallback_start_ = obs.now;
        phase_ = Phase::AWAIT_FALLBACK;
        return {XmAction::GLOBAL_FALLBACK, leg_index_, true};
      }
      return {XmAction::NONE, leg_index_, true};  // still relocalizing at the seed

    case Phase::AWAIT_FALLBACK:
      if (obs.reloc_locked) {
        ++leg_index_;
        phase_ = Phase::AWAIT_LEG;
        return {XmAction::SEND_LEG, leg_index_, false};
      }
      if (obs.now - fallback_start_ >= global_timeout_) {
        phase_ = Phase::FINISHED;
        return {XmAction::FINISH_FAIL, leg_index_, false};
      }
      return {XmAction::GLOBAL_FALLBACK, leg_index_, true};

    case Phase::FINISHED:
    default:
      return {XmAction::NONE, leg_index_, false};
  }
}

}  // namespace g1_map_manager
