#ifndef G1_LOCOMOTION_LOCO_BRIDGE_HPP
#define G1_LOCOMOTION_LOCO_BRIDGE_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

#include "g1_locomotion/joint_map.hpp"
#include "g1_locomotion/loco_safety.hpp"
#include "g1_locomotion/loco_types.hpp"

namespace g1_locomotion {

// Thread-safe wrapper over the G1 onboard high-level locomotion (LocoClient) and
// the LowState_ DDS stream. All public methods are safe to call concurrently.
class LocoBridge {
 public:
  struct Config {
    std::string network_interface;
    int domain_id = 0;
    double client_timeout = 0.2;      // LocoClient blocking-call timeout [s]
    VelocityLimits vel_limits;
    AccelLimits accel_limits;
    double command_ttl = 0.5;         // SetVelocity duration [s]
    double watchdog_timeout = 0.3;    // stale /cmd_vel threshold [s]
    VelocityDeadzone deadzone;        // per-axis /cmd_vel deadzone (>= 0)
    std::vector<int> allowed_fsm_ids; // walk FSMs where ROS may stream velocity
    std::vector<JointMapEntry> joint_map;
  };

  explicit LocoBridge(const Config& cfg);

  // Bring up DDS + LocoClient + LowState subscription. Returns false on failure;
  // lastError() then holds the reason. Call once before use.
  bool init();
  const std::string& lastError() const { return last_error_; }

  // /cmd_vel path: clamp then store as the target, stamping arrival time `now` [s].
  void setVelocityTarget(const Velocity& cmd, double now);

  // Control tick: pick desired (zero if halted or stale), slew applied toward it,
  // and always issue SetVelocity(applied, TTL). Returns the applied velocity.
  Velocity controlStep(double now, double dt);

  // Soft-stop / disarm (NOT damp): issues ONE SetVelocity(0) (prompt stop) and
  // latches halt (disarms streaming) -> the control loop then goes silent.
  // Exposed as /g1/halt. Returns the SDK SetVelocity code (0 == ok). Does NOT
  // switch the FSM; a deliberate release that drops the robot is /g1/set_mode.
  int halt();

  // Arm ROS velocity streaming (clears the halt latch). Exposed as /g1/arm. Does
  // NOT switch the FSM; the operator readies a walk FSM separately (remote or
  // /g1/set_mode walk_motion). controlStep streams only when armed AND the robot
  // is in a walk FSM (allowed_fsm_ids) AND a fresh /cmd_vel is active.
  void arm();

  // RESERVED MANUAL FSM switch (user-only, via /g1/set_mode). Switches the onboard
  // FSM: "walk_motion"->Start()(500, balanced), "lock_standing"->StandUp()(4),
  // "damping"->Damp()(1), "zero_torque"->ZeroTorque()(0). The latter three have NO
  // balance control (robot FALLS) and also disarm streaming; walk_motion does NOT
  // arm (use /g1/arm). The ONLY SetFsmId caller. Returns SDK code; fills `message`
  // and `drops_robot` (true for the three no-balance modes).
  int setMode(const std::string& mode, std::string& message, bool& drops_robot);

  // Latest joint positions from LowState. False if no LowState received yet.
  bool latestJointState(JointStateData& out) const;

  // Latest IMU from LowState. quat is [w,x,y,z] (SDK order). False if none yet.
  bool latestImu(std::array<double, 4>& quat, std::array<double, 3>& gyro,
                 std::array<double, 3>& accel) const;

  // Poll FSM id (blocking SDK call) -> LocoStatus mode + halt latch + error_code.
  // Also caches the FSM for the controlStep walk-FSM whitelist gate. Does NOT
  // auto-halt on an FSM change: controlStep stays silent off the walk FSMs.
  void pollStatus(std::uint8_t& mode, bool& halted, int& error_code);

 private:
  void onLowState(const void* msg);

  Config cfg_;
  std::string last_error_;

  // Constructed in init() AFTER ChannelFactory::Init: the Client base ctor binds
  // DDS, so building it before the factory is initialized segfaults.
  std::unique_ptr<unitree::robot::g1::LocoClient> client_;
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
      lowstate_sub_;
  std::mutex client_mutex_;  // serialize LocoClient calls

  mutable std::mutex state_mutex_;  // guards low_state_/have_state_
  unitree_hg::msg::dds_::LowState_ low_state_;
  bool have_state_ = false;

  std::mutex cmd_mutex_;  // guards target_/applied_/last_active_cmd_time_/
                          // halted_/last_fsm_id_
  Velocity target_;
  Velocity applied_;
  // Time [s] of the last non-zero (post-deadzone) /cmd_vel. Drives the "active"
  // check in controlStep; while inactive and stopped the bridge issues NO
  // SetVelocity (silent) so the manual remote keeps control.
  double last_active_cmd_time_ = -1.0;
  // Last FSM id observed by pollStatus, cached for the controlStep whitelist gate
  // (we stream velocity only when the robot is already in a walk FSM).
  int last_fsm_id_ = -1;
  // Soft-stop / halt latch (NOT damp). While true, controlStep commands zero
  // velocity and ignores /cmd_vel; in walk mode (FSM 500) the robot stays
  // balanced-standing. Boots true (latched-safe: no motion until walk_motion).
  bool halted_ = true;
};

}  // namespace g1_locomotion
#endif  // G1_LOCOMOTION_LOCO_BRIDGE_HPP
