#pragma once
// footprint_safety.h -- ROS-free footprint collision check + clear-distance query.
// The hard guarantee that the footprint never overlaps a lethal cell lives HERE, not
// in the QP: the MPC proposes, this component verifies the commanded trajectory
// against the real costmap and reports the distance to first collision.
//
// SafetyConfig is declared in robot_footprint.h (not here) so that plan_types.h
// can embed it without creating an include cycle through this file.

#include <utility>
#include <vector>

#include "g1_local_planner/core/costmap_view.h"
#include "g1_local_planner/core/plan_types.h"
#include "g1_local_planner/core/robot_footprint.h"

namespace g1_local_planner {

// Result of the safety filter: possibly clamped command + intervention flag + state.
struct SafetyResult {
  Twist2D         command;
  bool            intervened = false;
  ControllerState state      = ControllerState::TRACKING;
};

class FootprintSafety {
 public:
  static constexpr double kClearInf = 1e6;

  FootprintSafety(RobotFootprint fp, SafetyConfig cfg)
      : fp_(std::move(fp)), cfg_(cfg) {}

  const SafetyConfig& config() const { return cfg_; }
  const RobotFootprint& footprint() const { return fp_; }

  // True if any rotated footprint circle center lands on a cell >= lethal_value or
  // outside the grid.
  bool poseInCollision(const Pose2D& p, const CostmapView& cm) const;

  // Arc-length distance from `current` to the first colliding pose along the swept
  // predicted trajectory (subsampling between consecutive poses). Returns kClearInf
  // when the whole trajectory is clear.
  double clearDistance(const Pose2D& current,
                       const std::vector<Pose2D>& predicted,
                       const CostmapView& cm) const;

  // Verify/clamp the MPC command so the swept footprint stays clear and the robot can
  // always brake before contact. acc_lim_x is MpcConfig::acc_lim_x, control_dt the
  // 20 Hz control period (1/rate), NOT the MPC dt.
  SafetyResult filter(const Pose2D& current, const MpcResult& mpc,
                      const CostmapView& cm, double acc_lim_x,
                      double control_dt) const;

 private:
  bool worldPointLethal(double wx, double wy, const CostmapView& cm) const;

  // True if any cell whose CENTRE lies within `r` of (cx, cy) is lethal/unknown/
  // off-grid. Tests a footprint circle's full disk (not just its centre).
  bool worldDiskLethal(double cx, double cy, double r, const CostmapView& cm) const;

  RobotFootprint fp_;
  SafetyConfig   cfg_;
};

}  // namespace g1_local_planner
