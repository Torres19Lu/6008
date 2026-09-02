// Scan Context unit test.
//
// Covers the three behaviours the loop detector relies on:
//   1. a yaw-rotated + translated revisit of a stored scene is retrieved, and
//      the relative yaw is recovered (up to the SC sign ambiguity);
//   2. a structurally different scene is rejected (no match under threshold);
//   3. toScanContextFrame() aligns the height axis to gravity regardless of
//      base-link roll/pitch (the humanoid refinement).

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g1_slam_backend/scan_context.h"

using g1_slam_backend::ScanContextConfig;
using g1_slam_backend::ScanContextDB;

namespace {

// A vertical column of points at polar (r, theta) rising to height h. The bin's
// max-Z then encodes h, giving the descriptor distinctive structure.
void addLandmark(std::vector<Eigen::Vector3d>& pts, double r, double theta, double h) {
  const double x = r * std::cos(theta);
  const double y = r * std::sin(theta);
  for (double z = 0.0; z <= h + 1e-9; z += 0.25) {
    pts.emplace_back(x, y, z);
  }
}

std::vector<Eigen::Vector3d> rotateZ(const std::vector<Eigen::Vector3d>& pts, double phi) {
  const Eigen::Matrix3d r =
      Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::vector<Eigen::Vector3d> out;
  out.reserve(pts.size());
  for (const auto& p : pts) out.push_back(r * p);
  return out;
}

std::vector<Eigen::Vector3d> translate(const std::vector<Eigen::Vector3d>& pts,
                                       double dx, double dy) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(pts.size());
  for (const auto& p : pts) out.emplace_back(p.x() + dx, p.y() + dy, p.z());
  return out;
}

double angDiff(double a, double b) {
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

std::vector<Eigen::Vector3d> scene0() {
  std::vector<Eigen::Vector3d> p;
  addLandmark(p, 5.0, 0.3, 1.0);
  addLandmark(p, 8.0, 1.2, 2.0);
  addLandmark(p, 12.0, 2.5, 1.5);
  addLandmark(p, 6.0, -1.0, 3.0);
  addLandmark(p, 15.0, 3.0, 0.8);
  addLandmark(p, 9.0, -2.2, 2.5);
  return p;
}

}  // namespace

TEST(ScanContext, RetrievesRotatedTranslatedRevisit) {
  ScanContextDB db{ScanContextConfig()};
  db.add(0, db.make(scene0()));

  // Distractors at different ranges/heights/azimuths (separate keyframes).
  std::vector<Eigen::Vector3d> d1;
  addLandmark(d1, 30.0, 0.1, 0.5);
  addLandmark(d1, 45.0, 2.0, 1.2);
  addLandmark(d1, 20.0, -1.5, 2.2);
  db.add(1, db.make(d1));

  std::vector<Eigen::Vector3d> d2;
  addLandmark(d2, 60.0, 1.0, 1.8);
  addLandmark(d2, 35.0, -0.5, 0.9);
  addLandmark(d2, 50.0, 2.8, 2.6);
  db.add(2, db.make(d2));

  const double phi = 0.5236;  // 30 deg
  const std::vector<Eigen::Vector3d> revisit = translate(rotateZ(scene0(), phi), 0.2, 0.1);

  const ScanContextDB::QueryResult q =
      db.query(/*query_id=*/100, db.make(revisit), /*min_id_gap=*/10,
               /*dist_thresh=*/0.3, /*knn=*/5);

  ASSERT_TRUE(q.found);
  EXPECT_EQ(q.match_id, 0u);
  EXPECT_LT(q.distance, 0.15);
  // Recovered yaw matches the applied rotation up to the SC sign ambiguity,
  // within ~2 sector widths (2 * 2pi/60 ~ 0.21 rad).
  EXPECT_LT(std::min(angDiff(q.yaw, phi), angDiff(q.yaw, -phi)), 0.21);
}

TEST(ScanContext, RejectsDifferentScene) {
  ScanContextDB db{ScanContextConfig()};
  db.add(0, db.make(scene0()));

  std::vector<Eigen::Vector3d> d1;
  addLandmark(d1, 30.0, 0.1, 0.5);
  addLandmark(d1, 45.0, 2.0, 1.2);
  db.add(1, db.make(d1));

  // A scene at entirely different rings (ranges) -> no aligned column match.
  std::vector<Eigen::Vector3d> other;
  addLandmark(other, 40.0, 0.0, 2.0);
  addLandmark(other, 55.0, 1.6, 1.0);
  addLandmark(other, 70.0, -2.0, 2.8);
  addLandmark(other, 48.0, 2.9, 0.7);

  const ScanContextDB::QueryResult q =
      db.query(/*query_id=*/101, db.make(other), /*min_id_gap=*/10,
               /*dist_thresh=*/0.3, /*knn=*/5);

  EXPECT_FALSE(q.found);
}

TEST(ScanContext, ToScanContextFrameAlignsHeightToGravity) {
  // Base orientation in odom with nonzero roll, pitch, and yaw.
  const Eigen::Matrix3d odom_R_base =
      (Eigen::AngleAxisd(0.40, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();

  const Eigen::Vector3d p_odom(3.0, -2.0, 1.7);          // true height 1.7 m
  const Eigen::Vector3d p_base = odom_R_base.transpose() * p_odom;

  const std::vector<Eigen::Vector3d> out =
      ScanContextDB::toScanContextFrame({p_base}, odom_R_base);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_NEAR(out[0].z(), p_odom.z(), 1e-9);  // height aligned to gravity
  EXPECT_NEAR(std::hypot(out[0].x(), out[0].y()),
              std::hypot(p_odom.x(), p_odom.y()), 1e-9);  // horizontal radius kept
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
