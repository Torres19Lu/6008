#pragma once
// mode_machine.h -- the manager mode + the cross-map transition state machine. Pure
// (the caller supplies a monotonic `now`): per-leg send -> gateway dwell -> switch ->
// seed/fallback/timeout. ROS-free.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "g1_map_manager/core/map_types.h"
#include "g1_map_manager/core/topology_graph.h"

namespace g1_map_manager {

// What the node observed since the last step (edge events + monotonic clock).
struct ManagerObs {
  bool leg_succeeded = false;  // the active g1_nav leg finished OK
  bool leg_aborted = false;    // the active leg aborted
  bool reloc_locked = false;   // /slam/relocalize returned success
  bool reloc_failed = false;   // /slam/relocalize returned failure
  double now = 0.0;            // monotonic seconds
};

enum class XmAction : uint8_t {
  NONE = 0,
  SEND_LEG = 1,        // send legs[leg_index] to g1_nav
  DWELL_GATEWAY = 2,   // hold at the gateway (zero /cmd_vel, no backend call)
  SWITCH_MAP = 3,      // load next_map(read_only) + relocalize(seed)
  GLOBAL_FALLBACK = 4, // relocalize(global, no guess)
  FINISH_OK = 5,
  FINISH_FAIL = 6
};

struct XmDecision {
  XmAction action = XmAction::NONE;
  std::size_t leg_index = 0;
  bool transition = false;  // true while a cross-map switch is in progress
};

class ModeMachine {
 public:
  void setMode(Mode m) { mode_ = m; }
  Mode mode() const { return mode_; }
  bool canWriteMap() const { return mode_ != Mode::LOCALIZATION; }

  void setGlobalTimeout(double seconds) { global_timeout_ = seconds; }

  // transition_wait = effective gateway dwell (s) before the switch (the node resolves
  // the per-goal override vs the config default before calling this).
  void beginRoute(const std::vector<RouteLeg>& legs, double transition_wait,
                  double now);
  XmDecision step(const ManagerObs& obs);

  bool routeActive() const { return phase_ != Phase::FINISHED; }
  std::size_t legIndex() const { return leg_index_; }

 private:
  enum class Phase {
    SEND_LEG, AWAIT_LEG, DWELL, AWAIT_SEED, AWAIT_FALLBACK, FINISHED
  };
  XmDecision evalDwell(const ManagerObs& obs);

  Mode mode_ = Mode::IDLE;
  std::vector<RouteLeg> legs_;
  std::size_t leg_index_ = 0;
  double transition_wait_ = 0.0;
  double global_timeout_ = 15.0;
  double dwell_start_ = 0.0;
  double fallback_start_ = 0.0;
  double route_start_ = 0.0;
  Phase phase_ = Phase::FINISHED;
};

}  // namespace g1_map_manager
