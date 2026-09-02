#pragma once
// robot_footprint.h -- ROS-free body-frame footprint for g1_local_planner.
// The robot collision body is a lateral array of equal-radius circles centered on
// base_footprint (x=0,y=0). worldSamples() rotates the circle centers by yaw so the
// extended body width matters at the correct heading (orientation-aware collision).

#include <string>
#include <utility>
#include <vector>

namespace g1_local_planner {

struct FootprintCircle {
  double x_body = 0.0;
  double y_body = 0.0;
  double radius = 0.0;
};

// All fields have code defaults; the node YAML overrides. circle_y, when non-empty,
// overrides the evenly-spaced derivation (explicit body-frame Y centers).
struct FootprintConfig {
  double circle_radius = 0.15;   // m; half the forward depth (0.3 diameter)
  int    circle_count  = 5;      // number of circles when deriving centers
  double lateral_width = 0.5;    // m (Y); the body width the circle row spans
  double forward_depth = 0.30;   // m (X); documentation / nominal 2*circle_radius
  std::vector<double> circle_y;  // explicit body-frame Y centers (empty -> derive)
};

// Safety check parameters. Declared here (not in footprint_safety.h) so that
// MpcConfig (plan_types.h) can embed it without pulling in footprint_safety.h,
// which includes plan_types.h -- that would create a header cycle.
struct SafetyConfig {
  bool   enabled          = true;
  int    lethal_value     = 100;   // costmap cell value treated as collision (true-lethal)
  double brake_margin     = 0.05;  // m: braking-distance margin on the clear distance
  double footprint_collision_margin = 0.0;  // m: dilate each footprint circle for the
                                            // hard check (0 = pure footprint, honest zero)
  int    swept_subsamples = 2;     // interpolated checks between consecutive poses
  // When true, a cell with value -1 (unknown/unmapped) is treated as an obstacle.
  // Matches the local planner's treat_unknown_as_obstacle policy and the
  // "never overlap black/unknown" guarantee -- the footprint must not enter
  // unmapped space where collision geometry is undefined.
  bool   treat_unknown_as_obstacle = true;
};

class RobotFootprint {
 public:
  RobotFootprint() = default;

  // Build from cfg. Returns false and sets *err on invalid geometry
  // (count < 1, radius <= 0, non-finite). On success fills *out.
  static bool build(const FootprintConfig& cfg, RobotFootprint* out, std::string* err);

  double inscribedRadius() const { return inscribed_; }
  double circumscribedRadius() const { return circumscribed_; }
  const std::vector<FootprintCircle>& circles() const { return circles_; }

  // Rotate each circle center by yaw into the world frame, APPENDING (wx, wy) to
  // *out (callers reuse a buffer; this does not clear it).
  void worldSamples(double x, double y, double yaw,
                    std::vector<std::pair<double, double>>* out) const;

 private:
  std::vector<FootprintCircle> circles_;
  double inscribed_ = 0.0;
  double circumscribed_ = 0.0;
};

}  // namespace g1_local_planner
