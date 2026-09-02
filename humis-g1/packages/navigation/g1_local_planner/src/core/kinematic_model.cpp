#include "g1_local_planner/core/kinematic_model.h"

#include <cmath>

namespace g1_local_planner {

Eigen::Matrix3d controlMatrix(double theta0, double dt) {
  const double c  = std::cos(theta0);
  const double s_ = std::sin(theta0);

  Eigen::Matrix3d B;
  B << c * dt,  -s_ * dt,  0.0,
       s_ * dt,  c  * dt,  0.0,
       0.0,      0.0,       dt;
  return B;
}

Pose2D stepLinear(const Pose2D& s, const Twist2D& u, double theta0, double dt) {
  const Eigen::Matrix3d B = controlMatrix(theta0, dt);
  const Eigen::Vector3d uv(u.vx, u.vy, u.w);
  const Eigen::Vector3d delta = B * uv;

  Pose2D next;
  next.x   = s.x   + delta(0);
  next.y   = s.y   + delta(1);
  next.yaw = s.yaw + delta(2);
  return next;
}

Pose2D stepTrue(const Pose2D& s, const Twist2D& u, double dt) {
  const double c  = std::cos(s.yaw);
  const double s_ = std::sin(s.yaw);

  Pose2D next;
  next.x   = s.x   + (u.vx * c  - u.vy * s_) * dt;
  next.y   = s.y   + (u.vx * s_ + u.vy * c ) * dt;
  next.yaw = s.yaw + u.w * dt;
  return next;
}

std::vector<Pose2D> rolloutLinear(const Pose2D& s0,
                                  const std::vector<Twist2D>& us,
                                  double theta0,
                                  double dt) {
  std::vector<Pose2D> states;
  states.reserve(us.size());

  Pose2D cur = s0;
  for (const Twist2D& u : us) {
    cur = stepLinear(cur, u, theta0, dt);
    states.push_back(cur);
  }
  return states;
}

}  // namespace g1_local_planner
