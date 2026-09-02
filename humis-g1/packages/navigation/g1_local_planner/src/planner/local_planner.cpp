#include "g1_local_planner/planner/local_planner.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "g1_local_planner/core/footprint_safety.h"
#include "g1_local_planner/core/plan_types.h"
#include "g1_local_planner/core/robot_footprint.h"
#include "g1_local_planner/planner/path_reference.h"

namespace g1_local_planner {

LocalPlanner::LocalPlanner(const MpcConfig& cfg) : cfg_(cfg), mpc_(cfg) {
  rebuildFootprint();
}

void LocalPlanner::setConfig(const MpcConfig& cfg) {
  cfg_ = cfg;
  mpc_.setConfig(cfg);
  // A changed horizon invalidates the cached warm-start nominal (its size is
  // checked against cfg.horizon in solve()); reset the slew warm start too so the
  // first solve under the new limits starts from an in-box command.
  resetWarmStart();
  rebuildFootprint();
}

void LocalPlanner::rebuildFootprint() {
  std::string err;
  footprint_ok_ = RobotFootprint::build(cfg_.footprint, &footprint_, &err);
  // On invalid config the filter is skipped (footprint_ok_ == false); the node logs.
  if (footprint_ok_) mpc_.setFootprint(footprint_);
}

double LocalPlanner::maxFootprintCost(const Pose2D& p,
                                      const CostmapView& cm) const {
  if (!footprint_ok_ || !cm.valid()) return 0.0;
  std::vector<std::pair<double, double>> pts;
  footprint_.worldSamples(p.x, p.y, p.yaw, &pts);
  double m = 0.0;
  for (const auto& s : pts) m = std::max(m, cm.cost(s.first, s.second));
  return m;
}

void LocalPlanner::setPath(const std::vector<Pose2D>& path) {
  path_ = path;
  // New geometry: drop the obstacle-linearization nominal so the next solve does
  // not linearize about a trajectory from the old path. Keep last_cmd_ (the slew
  // anchor) so command continuity across a replan is preserved.
  last_predicted_.clear();
  // The path endpoint (stall-progress reference) moved; restart the progress watch.
  stall_counter_ = 0;
  stall_best_dist_ = 1e9;
}

void LocalPlanner::setGoal(const Pose2D& goal) {
  goal_ = goal;
  has_goal_ = true;
  last_predicted_.clear();  // see setPath: drop the stale warm-start nominal
  stall_counter_ = 0;       // new goal: restart the stall-progress watch
  stall_best_dist_ = 1e9;
}

void LocalPlanner::setCostmap(const CostmapView& costmap) { costmap_ = costmap; }

void LocalPlanner::clearGoal() {
  has_goal_ = false;
  resetWarmStart();
}

void LocalPlanner::resetWarmStart() {
  last_cmd_ = Twist2D{};
  last_predicted_.clear();
  stall_counter_ = 0;
  stall_best_dist_ = 1e9;
}

MpcResult LocalPlanner::compute(const Pose2D& current) {
  MpcResult result;

  // Helper: emit a zero-command stop result in the given state and reset the
  // open-loop warm start (so the next solve starts from rest). predicted stays
  // empty (no trajectory to publish for a stop).
  auto stop = [&](ControllerState state) -> MpcResult {
    resetWarmStart();
    MpcResult r;
    r.status = state;
    r.applied = Twist2D{};
    r.solved = false;
    return r;
  };

  // --- Precondition gates (precedence: goal, costmap, path) ---
  if (!has_goal_) {
    return stop(ControllerState::NO_GOAL);
  }
  if (!costmap_.valid()) {
    return stop(ControllerState::NO_COSTMAP);
  }
  if (path_.empty()) {
    return stop(ControllerState::NO_PATH);
  }

  // --- Reached check ---
  // Position is judged against the EFFECTIVE goal -- the path endpoint, which is
  // the planner's best reachable pose (the raw goal may sit inside an obstacle's
  // inscribed band and never be reachable). Heading is always the raw goal yaw.
  // reach_on_path_end == false reverts to the raw-goal check. path_ is guaranteed
  // non-empty here by the NO_PATH gate above, so path_.back() is safe.
  // A reachable goal (path endpoint within reach_goal_gap of the raw goal) is judged
  // against the RAW goal, so grid-snapping of the path endpoint does not cap accuracy.
  // An unreachable goal (endpoint far short, e.g. goal in an obstacle) falls back to
  // the path endpoint for graceful termination.
  const double end_to_goal =
      std::hypot(path_.back().x - goal_.x, path_.back().y - goal_.y);
  const bool goal_reachable = (end_to_goal <= cfg_.reach_goal_gap);
  const Pose2D& reach_target =
      (cfg_.reach_on_path_end && !goal_reachable) ? path_.back() : goal_;
  const double xy_err =
      std::hypot(reach_target.x - current.x, reach_target.y - current.y);
  const double yaw_err = wrapToPi(goal_.yaw - current.yaw);
  if (xy_err <= cfg_.xy_goal_tolerance &&
      std::abs(yaw_err) <= cfg_.yaw_goal_tolerance) {
    return stop(ControllerState::GOAL_REACHED);
  }

  // --- Arc-length rolling reference ---
  const std::vector<Pose2D> refs =
      generateReference(path_, current, goal_, cfg_);
  if (refs.empty()) {
    return stop(ControllerState::NO_PATH);
  }

  // --- MPC solve ---
  const double theta0 = current.yaw;  // LTI linearisation heading
  // Use last cycle's predicted path as the obstacle linearisation nominal only
  // when its size matches the horizon (i.e. the previous cycle solved). This is
  // the warm-started real-time-iteration SQP that sharpens avoidance across
  // cycles; solve() falls back to the reference points when nominal is null or
  // the wrong size.
  // With damping > 0, blend last cycle's predicted path toward the current
  // reference so the warm start cannot snap-back limit-cycle around a near-symmetric
  // block (Track C / C1). damping 0 = pure warm start. damped_nominal must outlive
  // the mpc_.solve call below (same scope).
  const std::vector<Pose2D>* nominal = nullptr;
  std::vector<Pose2D> damped_nominal;
  if (static_cast<int>(last_predicted_.size()) == cfg_.horizon) {
    const double d = cfg_.obstacle_nominal_damping;
    if (d <= 0.0) {
      nominal = &last_predicted_;
    } else {
      const double dd = (d > 1.0) ? 1.0 : d;
      damped_nominal.resize(last_predicted_.size());
      for (std::size_t k = 0; k < last_predicted_.size(); ++k) {
        const Pose2D& lp = last_predicted_[k];
        const Pose2D& rf = refs[k];  // refs.size() == horizon (non-empty, checked)
        damped_nominal[k].x   = (1.0 - dd) * lp.x + dd * rf.x;
        damped_nominal[k].y   = (1.0 - dd) * lp.y + dd * rf.y;
        damped_nominal[k].yaw = lp.yaw;  // yaw is not used by the gradient sampling
      }
      nominal = &damped_nominal;
    }
  }
  // Terminal obstacle relax: near a REACHABLE goal, suppress the soft obstacle term so
  // tracking can drive the center into the inscribed band where a safe-but-inscribed
  // goal sits; the post-MPC footprint safety filter remains the collision guarantee.
  double obstacle_scale = 1.0;
  if (goal_reachable && cfg_.goal_obstacle_relax_radius > 0.0 &&
      std::hypot(goal_.x - current.x, goal_.y - current.y) <
          cfg_.goal_obstacle_relax_radius) {
    obstacle_scale = cfg_.goal_obstacle_relax_scale;
  }
  result = mpc_.solve(current, theta0, refs, last_cmd_, costmap_, nominal,
                      obstacle_scale);

  if (result.solved) {
    // Carry the open-loop warm start forward: last_cmd seeds the next slew
    // constraint, last_predicted seeds the next obstacle nominal.
    last_cmd_ = result.applied;
    last_predicted_ = result.predicted;
    result.status = ControllerState::TRACKING;

    // Post-MPC footprint safety filter: hard guarantee that the swept footprint
    // stays clear of lethal cells and the robot can always brake before contact.
    // Skipped when the footprint config is invalid or the filter is disabled.
    if (footprint_ok_ && cfg_.safety.enabled) {
      FootprintSafety fs(footprint_, cfg_.safety);
      SafetyResult sr =
          fs.filter(current, result, costmap_, cfg_.acc_lim_x, cfg_.control_dt);
      result.applied = sr.command;
      // Reflect a hard stop in the state so g1_nav can react (-> RETRY).
      if (sr.state == ControllerState::STUCK) {
        result.status = ControllerState::STUCK;
      }
      // Keep last_cmd_ consistent with what we actually applied (slew continuity).
      last_cmd_ = result.applied;
    }

    // --- C3: suppress strafe near obstacles (anti-oscillation, Track C) ---
    // When the footprint sits near an obstacle, scale the applied lateral velocity
    // down so the robot prefers forward + yaw over side-to-side translation through
    // tight spaces. Gated on the max footprint-sample normalized cost.
    if (cfg_.vy_suppress_scale < 1.0 &&
        maxFootprintCost(current, costmap_) >= cfg_.vy_suppress_cost) {
      result.applied.vy *= cfg_.vy_suppress_scale;
      last_cmd_ = result.applied;  // keep the slew anchor consistent with what we apply
    }

    // --- Stall-near-end termination (Track A) ---
    // When the safety filter (or the approach ramp) holds the robot at the closest
    // pose it can safely reach near the path end, the controller stalls. Declare
    // GOAL_REACHED there instead of reporting STUCK (which would wrongly trigger
    // g1_nav recovery). Stall is judged by LACK OF PROGRESS toward the path end,
    // not low speed: a hard block makes the MPC limit-cycle, so the robot bounces
    // in place at nonzero speed without getting closer. The near-end gate (within
    // reach_stall_radius of the path endpoint) keeps a stall AWAY from the goal a
    // real block. The latch overrides a footprint-filter STUCK set just above.
    if (cfg_.reach_on_path_end) {
      const double dist_end = std::hypot(path_.back().x - current.x,
                                         path_.back().y - current.y);
      if (dist_end < cfg_.reach_stall_radius) {
        if (dist_end + cfg_.reach_stall_progress < stall_best_dist_) {
          stall_best_dist_ = dist_end;  // genuine progress toward the path end
          stall_counter_ = 0;
        } else {
          ++stall_counter_;             // near the end but not converging
        }
      } else {
        stall_counter_ = 0;
        stall_best_dist_ = 1e9;         // not near the end; reset the progress watch
      }
      if (stall_counter_ >= cfg_.reach_stall_cycles &&
          std::abs(wrapToPi(goal_.yaw - current.yaw)) <= cfg_.yaw_goal_tolerance) {
        return stop(ControllerState::GOAL_REACHED);
      }
    }

    return result;
  }

  // Solver failure: zero command, reset warm start, report STUCK.
  return stop(ControllerState::STUCK);
}

}  // namespace g1_local_planner
