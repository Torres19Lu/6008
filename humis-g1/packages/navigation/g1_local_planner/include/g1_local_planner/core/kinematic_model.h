#pragma once

// kinematic_model.h -- ROS-free LTI kinematic model for g1_local_planner.
// No ROS headers. Uses Eigen (Eigen::Matrix3d / Vector3d).
//
// Model is LTI (linear time-invariant) linearised about a FIXED heading theta0.
// Over the entire MPC horizon, the body-to-map rotation is held at
// R(theta0) so body velocity -> map displacement is linear and the resulting
// QP is strictly convex.
//
// Notation
//   s = [x, y, yaw]  (map frame, metres / radians)
//   u = [vx, vy, w]  (body-frame velocity: forward, lateral, yaw rate)
//   theta0 = FIXED linearisation heading (current robot yaw from TF)
//   c = cos(theta0),  s_ = sin(theta0)
//
// LTI step (what the QP predicts and RViz shows):
//   x_{k+1}   = x_k   + (vx*c  - vy*s_) * dt
//   y_{k+1}   = y_k   + (vx*s_ + vy*c ) * dt
//   yaw_{k+1} = yaw_k + w * dt
//   i.e.  s_{k+1} = s_k + B(theta0, dt) * u_k
//   with  B = [[c*dt, -s_*dt, 0],
//               [s_*dt,  c*dt, 0],
//               [0,      0,   dt]]
//
// stepLinear / rolloutLinear use this LTI step (same fixed theta0 for all steps).
// They are the QP's own prediction model and match what RViz shows.
//
// stepTrue uses the TRUE nonlinear kinematics (rotates by the actual yaw at each
// step). It is provided ONLY for the offline closed-loop simulation gate (Task 9)
// and must NOT be used in the QP formulation.
//
// NOTE: yaw is NOT normalised here; consumers normalise when comparing angles.

#include <vector>

#include <Eigen/Core>

#include "g1_local_planner/core/plan_types.h"

namespace g1_local_planner {

// Returns the 3x3 control-input matrix B(theta0, dt) such that
//   s_{k+1} = s_k + B * u_k
// where s = [x, y, yaw]^T and u = [vx, vy, w]^T (body frame).
Eigen::Matrix3d controlMatrix(double theta0, double dt);

// LTI step: apply body velocity u over dt using the FIXED linearisation heading
// theta0. This is the step used by the QP's prediction model.
//   s_{k+1} = s_k + B(theta0, dt) * u
Pose2D stepLinear(const Pose2D& s, const Twist2D& u, double theta0, double dt);

// Nonlinear (true plant) step: rotate by the ACTUAL yaw of s at each step.
//   x_{k+1}   = x_k   + (vx*cos(s.yaw) - vy*sin(s.yaw)) * dt
//   y_{k+1}   = y_k   + (vx*sin(s.yaw) + vy*cos(s.yaw)) * dt
//   yaw_{k+1} = yaw_k + w * dt
// Used ONLY by the offline closed-loop simulation (Task 9). Do NOT use in the QP.
Pose2D stepTrue(const Pose2D& s, const Twist2D& u, double dt);

// Rolls out the LTI model for N steps starting from s0, applying each u in us,
// using the SAME fixed theta0 throughout.
// Returns s_1 .. s_N (size == us.size()); each step uses the previous output.
std::vector<Pose2D> rolloutLinear(const Pose2D& s0,
                                  const std::vector<Twist2D>& us,
                                  double theta0,
                                  double dt);

}  // namespace g1_local_planner
