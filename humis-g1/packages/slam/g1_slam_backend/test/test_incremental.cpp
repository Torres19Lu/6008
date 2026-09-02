// Incremental-append unit tests for the backend core.
//
// Two additive hooks let a loaded (frozen) map resume MAPPING after a
// relocalization lock:
//   reset()           - clear all state back to a fresh mapping backend.
//   beginIncremental() - re-base each loaded keyframe's odom_pose into the live
//                        odom frame using the relocalized map->odom, so the FIRST
//                        new keyframe's bridge factor is correct.
//
// loadMap stores each keyframe's odom_pose as a PLACEHOLDER (the optimized map
// pose), so a naive "set MAPPING and append" makes the bridge factor garbage.
// beginIncremental fixes that: with map_to_odom = T, it sets
//   kf.odom_pose := T^-1 * optimizedPose(id)
// so a new live frame P_new lands at T * P_new in the map frame.

#include <gtest/gtest.h>

#include <filesystem>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "g1_slam_backend/backend.h"

using g1_slam_backend::Backend;
using g1_slam_backend::BackendConfig;

namespace {

namespace fs = std::filesystem;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

Eigen::Isometry3d xyPose(double x, double y) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << x, y, 0.0;
  return T;
}

const std::vector<Eigen::Vector3d>& landmarks() {
  static const std::vector<Eigen::Vector3d> lm = {
      {3, 2, 1.0},   {7, -1, 2.0}, {-4, 5, 1.5}, {10, 3, 0.8},
      {-6, -3, 2.5}, {5, 8, 1.2},  {-8, 1, 1.8}, {2, -7, 2.2}};
  return lm;
}

Cloud::Ptr worldCloud(double sx, double sy) {
  Cloud::Ptr c(new Cloud());
  for (const Eigen::Vector3d& l : landmarks()) {
    for (double z = 0.0; z <= l.z() + 1e-9; z += 0.25) {
      pcl::PointXYZI p;
      p.x = static_cast<float>(l.x() + sx);
      p.y = static_cast<float>(l.y() + sy);
      p.z = static_cast<float>(z);
      p.intensity = 1.0f;
      c->push_back(p);
    }
  }
  return c;
}

BackendConfig loopConfig() {
  BackendConfig cfg;
  cfg.keyframe_dist = 1.0;
  cfg.keyframe_angle = 0.5;
  cfg.keyframe_voxel = 0.0;
  cfg.map_voxel = 0.0;
  cfg.sc_min_id_gap = 3;
  cfg.sc_dist_thresh = 0.2;
  cfg.sc_knn = 5;
  cfg.icp_max_corr_dist = 3.0;
  cfg.icp_max_iter = 50;
  cfg.icp_fitness_thresh = 0.5;
  cfg.icp_submap_neighbors = 0;
  cfg.loop_icp_voxel = 0.0;
  cfg.loop_rot_sigma = 0.02;
  cfg.loop_trans_sigma = 0.05;
  cfg.loop_extra_iters = 10;
  cfg.loop_robust_c = 0.0;  // convex: an incremental build matches a batch reload
  cfg.loop_max_dt = 1e9;
  cfg.loop_max_dr = 1e9;
  return cfg;
}

void buildLoopedBackend(Backend* backend) {
  const std::vector<Eigen::Vector2d> truth = {
      {0, 0}, {2, 0}, {4, 0}, {4, 2}, {4, 4}, {2, 4}, {0, 4}, {0, 2}, {0, 0}};
  const double drift_per_step = 0.08;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    const double d = drift_per_step * static_cast<double>(i);
    const Eigen::Isometry3d odom = xyPose(truth[i].x() + d, truth[i].y() + d);
    const Cloud::Ptr cw = worldCloud(d, d);
    backend->stepIntake(static_cast<double>(i), odom, *cw);
  }
  for (int i = 0; i < 50 && backend->hasPendingLoopChecks(); ++i) {
    backend->runLoopClosureOnce();
  }
}

// A simulated relocalization lock: a non-trivial map->odom (yaw + translation).
Eigen::Isometry3d simulatedLock() {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() << 10.0, -5.0, 0.0;
  return T;
}

}  // namespace

TEST(Incremental, ResetClearsAllState) {
  Backend backend(loopConfig());
  buildLoopedBackend(&backend);
  ASSERT_GE(backend.numKeyframes(), 5u);
  ASSERT_FALSE(backend.loopEdges().empty());

  backend.reset();

  EXPECT_EQ(backend.numKeyframes(), 0u);
  EXPECT_TRUE(backend.loopEdges().empty());
  EXPECT_FALSE(backend.hasPendingLoopChecks());
  EXPECT_TRUE(backend.mapToOdom().isApprox(Eigen::Isometry3d::Identity(), 1e-9));

  // The cleared backend builds a fresh first keyframe (graph rebuilt, no leftovers).
  EXPECT_TRUE(backend.stepIntake(0.0, xyPose(0, 0), *worldCloud(0, 0)));
  EXPECT_EQ(backend.numKeyframes(), 1u);
}

TEST(Incremental, BeginIncrementalRebasesAppendIntoMapFrame) {
  // Build + save a map.
  Backend src(loopConfig());
  buildLoopedBackend(&src);
  const fs::path dir = fs::temp_directory_path() / "g1_slam_backend_incr_test";
  std::error_code ec;
  fs::remove_all(dir, ec);
  std::string msg;
  ASSERT_TRUE(src.saveMap(dir.string(), &msg)) << msg;

  // Load into a fresh backend; simulate a relocalization lock.
  Backend b(loopConfig());
  ASSERT_TRUE(b.loadMap(dir.string(), &msg)) << msg;
  const Eigen::Isometry3d T = simulatedLock();
  b.setMapToOdom(T);
  const std::size_t n = b.numKeyframes();
  const Eigen::Isometry3d x_last = b.optimizedPoses().back();  // map pose of kf n-1

  b.beginIncremental();

  // A live odom frame P_new = (T^-1 * x_last) * M, M = 1.5 m forward (> keyframe_dist).
  Eigen::Isometry3d m = Eigen::Isometry3d::Identity();
  m.translation() << 1.5, 0.0, 0.0;
  const Eigen::Isometry3d p_new = (T.inverse() * x_last) * m;
  ASSERT_TRUE(b.stepIntake(100.0, p_new, *worldCloud(1.0, 1.0)));
  ASSERT_EQ(b.numKeyframes(), n + 1);

  // The new keyframe lands at x_last * M = T * P_new in the map frame, and the
  // re-base keeps map->odom consistent at the lock T.
  const Eigen::Isometry3d expected = x_last * m;
  const Eigen::Isometry3d got = b.optimizedPoses().back();
  EXPECT_LT((got.translation() - expected.translation()).norm(), 3e-2);
  EXPECT_LT((b.mapToOdom().translation() - T.translation()).norm(), 3e-2);

  // Contrast: WITHOUT beginIncremental the placeholder odom_pose makes the append
  // land far from the correct map pose (off by ~the lock translation).
  Backend naive(loopConfig());
  ASSERT_TRUE(naive.loadMap(dir.string(), &msg)) << msg;
  naive.setMapToOdom(T);
  ASSERT_TRUE(naive.stepIntake(100.0, p_new, *worldCloud(1.0, 1.0)));
  const Eigen::Isometry3d naive_got = naive.optimizedPoses().back();
  EXPECT_GT((naive_got.translation() - expected.translation()).norm(), 1.0)
      << "naive append must be wrong without the re-base";

  fs::remove_all(dir, ec);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
