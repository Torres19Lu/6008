#include "g1_locomotion/loco_bridge.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <string>

#include <g1_msgs/LocoStatus.h>
#include <unitree/robot/channel/channel_factory.hpp>

namespace g1_locomotion {
namespace {

std::string toLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return s;
}

}  // namespace

LocoBridge::LocoBridge(const Config& cfg) : cfg_(cfg) {}

bool LocoBridge::init() {
  try {
    unitree::robot::ChannelFactory::Instance()->Init(cfg_.domain_id,
                                                     cfg_.network_interface);
  } catch (const std::exception& e) {
    last_error_ = std::string("ChannelFactory Init failed: ") + e.what();
    return false;
  }
  try {
    // Construct AFTER ChannelFactory::Init: the Client base ctor binds DDS, so
    // building LocoClient before the factory is initialized segfaults.
    client_ = std::make_unique<unitree::robot::g1::LocoClient>();
    client_->Init();
    client_->SetTimeout(static_cast<float>(cfg_.client_timeout));
  } catch (const std::exception& e) {
    last_error_ = std::string("LocoClient Init failed: ") + e.what();
    return false;
  }
  try {
    lowstate_sub_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            "rt/lowstate"));
    lowstate_sub_->InitChannel([this](const void* msg) { onLowState(msg); }, 1);
  } catch (const std::exception& e) {
    last_error_ = std::string("LowState subscribe failed: ") + e.what();
    return false;
  }
  return true;
}

void LocoBridge::onLowState(const void* msg) {
  const auto* s = static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
  std::lock_guard<std::mutex> lock(state_mutex_);
  low_state_ = *s;
  have_state_ = true;
}

void LocoBridge::setVelocityTarget(const Velocity& cmd, double now) {
  // Deadzone first (sub-threshold -> 0 so it reads as "no command"), then clamp.
  const Velocity shaped =
      clampVelocity(applyDeadzone(cmd, cfg_.deadzone), cfg_.vel_limits);
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  target_ = shaped;
  // Only a non-zero command counts as "active": a stream of (deadzoned) zeros is
  // idle, so the bridge falls silent and the manual remote keeps control.
  if (shaped.vx != 0.0 || shaped.vy != 0.0 || shaped.vyaw != 0.0) {
    last_active_cmd_time_ = now;
  }
}

Velocity LocoBridge::controlStep(double now, double dt) {
  Velocity applied;
  bool active;
  bool fsm_ok;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    fsm_ok = fsmAllowed(last_fsm_id_, cfg_.allowed_fsm_ids);
    const double age =
        (last_active_cmd_time_ < 0.0) ? 1e9 : (now - last_active_cmd_time_);
    active = !halted_ && fsm_ok && !isCommandStale(age, cfg_.watchdog_timeout);
    const Velocity desired = active ? target_ : Velocity{};  // else decel to 0
    applied_ = slewVelocity(applied_, desired, cfg_.accel_limits, dt);
    applied = applied_;
  }
  // Send SetVelocity only while actively driving OR still decelerating to a
  // stop, AND only when the robot is in an allowed walk FSM. Once idle/stopped,
  // or whenever the robot is not in a walk FSM, go SILENT (issue nothing) so the
  // Unitree handheld remote keeps control of the shared "sport" service. We do
  // NOT force the FSM; command_ttl is the crash backstop. NEVER damps.
  // SetVelocity (not Move): Move only yields duration 1.0s or 864000s.
  constexpr double kEps = 1e-4;
  const bool moving = std::abs(applied.vx) > kEps ||
                      std::abs(applied.vy) > kEps ||
                      std::abs(applied.vyaw) > kEps;
  if (fsm_ok && (active || moving)) {
    std::lock_guard<std::mutex> lock(client_mutex_);
    if (client_) {
      client_->SetVelocity(static_cast<float>(applied.vx),
                           static_cast<float>(applied.vy),
                           static_cast<float>(applied.vyaw),
                           static_cast<float>(cfg_.command_ttl));
    }
  }
  return applied;
}

int LocoBridge::halt() {
  // Soft-stop, NOT damp: zero the velocity and latch halt so the robot stays
  // balanced-standing in walk mode (FSM 500). A deliberate release (no balance
  // control -> the robot FALLS) is a set_mode action, never the e-stop.
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    halted_ = true;
    target_ = Velocity{};
    applied_ = Velocity{};
  }
  std::lock_guard<std::mutex> lock(client_mutex_);
  if (!client_) return -1;
  return client_->SetVelocity(0.f, 0.f, 0.f,
                              static_cast<float>(cfg_.command_ttl));
}

void LocoBridge::arm() {
  // Enable ROS velocity streaming (clear the halt latch). Does NOT switch the
  // FSM; controlStep still streams only when the robot is in a walk FSM and a
  // fresh /cmd_vel is active. Require a fresh /cmd_vel before any motion.
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  halted_ = false;
  target_ = Velocity{};
  applied_ = Velocity{};
  last_active_cmd_time_ = -1.0;
}

int LocoBridge::setMode(const std::string& mode, std::string& message,
                        bool& drops_robot) {
  // Normalize: lowercase, spaces/hyphens -> underscores.
  std::string m = toLower(mode);
  std::replace(m.begin(), m.end(), ' ', '_');
  std::replace(m.begin(), m.end(), '-', '_');

  // RESERVED MANUAL FSM switch (user-only, via /g1/set_mode). This is the ONLY
  // code that calls SetFsmId; NO startup/timer/watchdog/halt/control path ever
  // switches the FSM. It does NOT arm streaming (that is /g1/arm); the three
  // no-balance modes additionally disarm (the robot is dropping).
  drops_robot = false;
  int ret = -1;
  int new_fsm = -1;
  std::lock_guard<std::mutex> lock(client_mutex_);
  if (!client_) {
    message = "locomotion client not initialized";
    return -1;
  }
  if (m == "walk_motion" || m == "walk" || m == "start") {
    ret = client_->Start();  // FSM 500: balanced; accepts velocity
    new_fsm = 500;
    message = "walk_motion (FSM 500); arm with /g1/arm to drive";
  } else if (m == "lock_standing" || m == "stand") {
    ret = client_->StandUp();  // FSM 4
    new_fsm = 4;
    drops_robot = true;
    message = "lock_standing (WARNING: no balance control, robot will fall)";
  } else if (m == "damping" || m == "damp") {
    ret = client_->Damp();  // FSM 1
    new_fsm = 1;
    drops_robot = true;
    message = "damping (WARNING: no balance control, robot will fall)";
  } else if (m == "zero_torque" || m == "zerotorque" || m == "zero") {
    ret = client_->ZeroTorque();  // FSM 0
    new_fsm = 0;
    drops_robot = true;
    message = "zero_torque (WARNING: no balance control, robot will fall)";
  } else {
    message = "unknown mode '" + mode +
              "' (use walk_motion|lock_standing|damping|zero_torque)";
    return -1;
  }
  if (ret == 0) {
    std::lock_guard<std::mutex> l(cmd_mutex_);
    // Reflect the commanded FSM immediately so controlStep's walk-FSM whitelist
    // gate updates without waiting for the next pollStatus.
    last_fsm_id_ = new_fsm;
    if (drops_robot) {  // a deliberate drop also disarms ROS streaming
      halted_ = true;
      target_ = Velocity{};
      applied_ = Velocity{};
    }
  } else {
    message += " (SDK error " + std::to_string(ret) + ")";
  }
  return ret;
}

bool LocoBridge::latestJointState(JointStateData& out) const {
  std::array<double, kLowStateMotorCount> q{};
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!have_state_) return false;
    for (std::size_t i = 0; i < kLowStateMotorCount; ++i) {
      q[i] = static_cast<double>(low_state_.motor_state()[i].q());
    }
  }
  out = buildJointState(q, cfg_.joint_map);
  return true;
}

bool LocoBridge::latestImu(std::array<double, 4>& quat,
                           std::array<double, 3>& gyro,
                           std::array<double, 3>& accel) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!have_state_) return false;
  const auto& imu = low_state_.imu_state();
  for (int i = 0; i < 4; ++i) quat[i] = static_cast<double>(imu.quaternion()[i]);
  for (int i = 0; i < 3; ++i) gyro[i] = static_cast<double>(imu.gyroscope()[i]);
  for (int i = 0; i < 3; ++i) {
    accel[i] = static_cast<double>(imu.accelerometer()[i]);
  }
  return true;
}

void LocoBridge::pollStatus(std::uint8_t& mode, bool& halted, int& error_code) {
  int fsm_id = -1;
  int ret;
  {
    std::lock_guard<std::mutex> lock(client_mutex_);
    ret = client_ ? client_->GetFsmId(fsm_id) : -1;
  }
  const bool ok = (ret == 0);
  mode = ok ? fsmIdToLocoMode(fsm_id) : g1_msgs::LocoStatus::MODE_UNKNOWN;
  error_code = ret;
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  // Cache the FSM for the controlStep whitelist gate (only on a confirmed read;
  // a failed poll leaves the last good value). We do NOT auto-halt on an FSM
  // change: controlStep simply stays silent when the robot is not in a walk FSM
  // (yields to the remote) and resumes when it returns. `halted` reports the
  // soft-stop/halt latch (true after /g1/halt or a non-balancing set_mode).
  if (ok) last_fsm_id_ = fsm_id;
  halted = halted_;
}

}  // namespace g1_locomotion
