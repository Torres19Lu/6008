// Relocalization unit test.
//
// Build a small synthetic map (keyframes at known poses through an asymmetric
// landmark field, no drift so optimized poses == ground truth). Then simulate a
// fresh session whose odom frame is offset from the map by a known map->odom:
//   - relocalizeGlobal recovers map->odom from an unknown start (same heading,
//     and with a large in-place heading offset that exercises the SC yaw seed),
//   - relocalizeLocal refines map->odom from a perturbed prior (tracking).

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "g1_slam_backend/backend.h"

using g1_slam_backend::Backend;
using g1_slam_backend::BackendConfig;

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

Eigen::Isometry3d pose(double x, double y, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() << x, y, 0.0;
  return T;
}

// Asymmetric vertical-column landmark field, in the world (== map) frame.
Cloud::Ptr worldField() {
  static const std::vector<Eigen::Vector3d> lm = {
      {3, 2, 1.0},   {7, -1, 2.0}, {-4, 5, 1.5}, {10, 3, 0.8},
      {-6, -3, 2.5}, {5, 8, 1.2},  {-8, 1, 1.8}, {2, -7, 2.2}};
  Cloud::Ptr c(new Cloud());
  for (const Eigen::Vector3d& l : lm) {
    for (double z = 0.0; z <= l.z() + 1e-9; z += 0.25) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(l.x());
      p.y = static_cast<float>(l.y());
      p.z = static_cast<float>(z);
      p.intensity = 1.0f;
      c->push_back(p);
    }
  }
  return c;
}

Cloud::Ptr transformCloud(const Eigen::Isometry3d& T, const Cloud& in) {
  Cloud::Ptr out(new Cloud());
  pcl::transformPointCloud(in, *out, T.matrix().cast<float>());
  return out;
}

BackendConfig relocConfig() {
  BackendConfig cfg;
  cfg.keyframe_dist = 1.0;
  cfg.keyframe_angle = 0.5;
  cfg.keyframe_voxel = 0.0;  // exact synthetic clouds
  cfg.map_voxel = 0.0;
  cfg.sc_dist_thresh = 0.3;
  cfg.sc_knn = 5;
  cfg.icp_max_corr_dist = 5.0;
  cfg.icp_max_iter = 60;
  cfg.icp_fitness_thresh = 0.5;
  cfg.icp_submap_neighbors = 0;
  return cfg;
}

// Build the map and return ground-truth keyframe poses (== optimized poses).
std::vector<Eigen::Isometry3d> buildMap(Backend* backend) {
  const std::vector<Eigen::Isometry3d> truth = {
      pose(0, 0, 0.0),  pose(2, 0, 0.0), pose(4, 0, 0.0),
      pose(4, 2, 0.0),  pose(4, 4, 0.0), pose(2, 4, 0.0)};
  const Cloud::Ptr field = worldField();
  for (std::size_t i = 0; i < truth.size(); ++i) {
    EXPECT_TRUE(backend->stepIntake(static_cast<double>(i), truth[i], *field));
  }
  return truth;
}

void expectClose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b,
                 double trans_tol, double rot_tol) {
  EXPECT_LT((a.translation() - b.translation()).norm(), trans_tol);
  const Eigen::AngleAxisd d(a.rotation().transpose() * b.rotation());
  EXPECT_LT(std::abs(d.angle()), rot_tol);
}

}  // namespace

TEST(Relocalize, GlobalAcquireFromUnknownStart) {
  Backend backend(relocConfig());
  buildMap(&backend);
  ASSERT_EQ(backend.numKeyframes(), 6u);

  // Unknown map->odom offset for the fresh session, and a target place == kf 2.
  const Eigen::Isometry3d truth = pose(5.0, -3.0, 0.6);
  const Eigen::Isometry3d x_m = backend.optimizedPoses()[2];

  const Eigen::Isometry3d p_now = truth.inverse() * x_m;  // live odom pose
  const Cloud::Ptr live = transformCloud(truth.inverse(), *worldField());

  const Backend::RelocResult r = backend.relocalizeGlobal(p_now, *live);
  ASSERT_TRUE(r.found);
  EXPECT_EQ(r.match_id, 2u);
  expectClose(r.map_to_odom, truth, 0.2, 0.05);
}

TEST(Relocalize, GlobalAcquireWithHeadingOffset) {
  Backend backend(relocConfig());
  buildMap(&backend);

  const Eigen::Isometry3d truth = pose(-2.0, 4.0, -0.9);
  const Eigen::Isometry3d x_m = backend.optimizedPoses()[2];
  // Robot at kf 2's position but rotated ~46 deg in place: exercises the SC yaw.
  const Eigen::Isometry3d x_target = x_m * pose(0, 0, 0.8);

  const Eigen::Isometry3d p_now = truth.inverse() * x_target;
  const Cloud::Ptr live = transformCloud(truth.inverse(), *worldField());

  const Backend::RelocResult r = backend.relocalizeGlobal(p_now, *live);
  ASSERT_TRUE(r.found);
  expectClose(r.map_to_odom, truth, 0.2, 0.05);
}

TEST(Relocalize, LocalTrackingRefinesPerturbedPrior) {
  Backend backend(relocConfig());
  buildMap(&backend);

  const Eigen::Isometry3d truth = pose(3.0, 1.0, 0.3);
  const Eigen::Isometry3d x_m = backend.optimizedPoses()[2];
  const Eigen::Isometry3d p_now = truth.inverse() * x_m;
  const Cloud::Ptr live = transformCloud(truth.inverse(), *worldField());

  // A prior off the true pose by ~0.36 m and ~0.1 rad (odom drift since lock).
  const Eigen::Isometry3d x_pred = x_m * pose(0.3, -0.2, 0.1);

  const Backend::RelocResult r =
      backend.relocalizeLocal(p_now, *live, x_pred, /*radius=*/50.0);
  ASSERT_TRUE(r.found);
  expectClose(r.map_to_odom, truth, 0.1, 0.03);
}

TEST(Relocalize, TrackAgainstCachedSubmap) {
  Backend backend(relocConfig());
  buildMap(&backend);

  const Eigen::Isometry3d truth = pose(1.0, -2.0, 0.4);
  const Eigen::Isometry3d x_m = backend.optimizedPoses()[2];
  const Eigen::Isometry3d p_now = truth.inverse() * x_m;
  const Cloud::Ptr live = transformCloud(truth.inverse(), *worldField());

  // A perturbed prior, and the cached map-frame submap the node would build once.
  const Eigen::Isometry3d x_pred = x_m * pose(0.3, -0.2, 0.1);
  const std::vector<Backend::KeyframeView> near =
      backend.snapshotKeyframesNear(x_pred.translation(), 50.0);
  const Cloud::Ptr submap = Backend::assembleFromSnapshot(near, 0.0);
  ASSERT_FALSE(submap->empty());

  const Backend::RelocResult r =
      backend.trackAgainstSubmap(p_now, *live, x_pred, submap);
  ASSERT_TRUE(r.found);
  expectClose(r.map_to_odom, truth, 0.1, 0.03);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
