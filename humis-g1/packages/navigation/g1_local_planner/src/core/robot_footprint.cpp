// robot_footprint.cpp -- ROS-free body-frame footprint for g1_local_planner.
#include "g1_local_planner/core/robot_footprint.h"

#include <algorithm>
#include <cmath>

namespace g1_local_planner {

bool RobotFootprint::build(const FootprintConfig& cfg, RobotFootprint* out,
                           std::string* err) {
  auto setErr = [&](const std::string& m) { if (err) *err = m; return false; };
  if (out == nullptr) return false;
  if (!(cfg.circle_radius > 0.0) || !std::isfinite(cfg.circle_radius)) {
    return setErr("footprint circle_radius must be > 0");
  }

  std::vector<double> ys;
  if (!cfg.circle_y.empty()) {
    ys = cfg.circle_y;
  } else {
    if (cfg.circle_count < 1) return setErr("footprint circle_count must be >= 1");
    const double half_span = cfg.lateral_width / 2.0 - cfg.circle_radius;
    if (cfg.circle_count == 1 || half_span <= 0.0) {
      ys.assign(static_cast<size_t>(cfg.circle_count == 1 ? 1 : cfg.circle_count),
                0.0);
      if (cfg.circle_count > 1) {
        // half_span <= 0: circles wider than the body; stack them all at center.
        ys.assign(static_cast<size_t>(cfg.circle_count), 0.0);
      } else {
        ys = {0.0};
      }
    } else {
      ys.resize(static_cast<size_t>(cfg.circle_count));
      const int n = cfg.circle_count;
      for (int i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n - 1);
        ys[static_cast<size_t>(i)] = -half_span + t * (2.0 * half_span);
      }
    }
  }

  out->circles_.clear();
  out->circles_.reserve(ys.size());
  out->inscribed_ = cfg.circle_radius;
  double circ = 0.0;
  for (double y : ys) {
    if (!std::isfinite(y)) return setErr("footprint circle_y has non-finite value");
    FootprintCircle c;
    c.x_body = 0.0;
    c.y_body = y;
    c.radius = cfg.circle_radius;
    out->circles_.push_back(c);
    circ = std::max(circ, std::hypot(c.x_body, c.y_body) + c.radius);
  }
  if (out->circles_.empty()) return setErr("footprint has no circles");
  out->circumscribed_ = circ;

  // Coverage invariant: the circles (a lateral row at x_body=0) must overlap so
  // their disks form a gap-free cover of the declared body. Adjacent centers (by y)
  // no farther apart than 2*radius. This makes a zero-margin collision check honest
  // against the declared 0.5x0.3 stadium body.
  std::vector<double> ys_sorted;
  ys_sorted.reserve(out->circles_.size());
  for (const auto& cc : out->circles_) ys_sorted.push_back(cc.y_body);
  std::sort(ys_sorted.begin(), ys_sorted.end());
  for (std::size_t i = 1; i < ys_sorted.size(); ++i) {
    if (ys_sorted[i] - ys_sorted[i - 1] > 2.0 * cfg.circle_radius + 1e-9) {
      return setErr("footprint circles leave a lateral gap (adjacent spacing > 2*radius)");
    }
  }
  return true;
}

void RobotFootprint::worldSamples(double x, double y, double yaw,
                                  std::vector<std::pair<double, double>>* out) const {
  if (out == nullptr) return;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  out->reserve(out->size() + circles_.size());
  for (const auto& circle : circles_) {
    const double dx = c * circle.x_body - s * circle.y_body;
    const double dy = s * circle.x_body + c * circle.y_body;
    out->emplace_back(x + dx, y + dy);
  }
}

}  // namespace g1_local_planner
