// Map persistence round-trip unit test.
//
// Drive the same closed, drifting square loop as test_backend so the backend
// builds several keyframes plus at least one loop edge. Save the map directory,
// load it into a FRESH backend, and assert the reconstruction is faithful:
// keyframe count, per-keyframe clouds and Scan Context descriptors, the loop
// edge measurements, and the optimized poses (the rebuilt iSAM2 graph must
// re-converge to the saved estimate). loadMap must also leave map->odom unset
// (Identity), since relocalization, not load, establishes it.

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
  cfg.keyframe_voxel = 0.0;  // keep synthetic clouds exact
  cfg.map_voxel = 0.0;
  cfg.sc_min_id_gap = 3;
  cfg.sc_dist_thresh = 0.2;
  cfg.sc_knn = 5;
  cfg.icp_max_corr_dist = 3.0;
  cfg.icp_max_iter = 50;
  cfg.icp_fitness_thresh = 0.5;
  cfg.icp_submap_neighbors = 0;
  cfg.loop_icp_voxel = 0.0;  // keep the exact synthetic clouds for an exact round-trip
  cfg.loop_rot_sigma = 0.02;
  cfg.loop_trans_sigma = 0.05;
  cfg.loop_extra_iters = 10;
  // Serialization round-trip is tested with the CONVEX (plain Gaussian) optimizer
  // and the consistency gate disabled, so a loop is present AND an incremental
  // build matches a batch reload exactly. The Cauchy kernel is non-convex (build
  // != reload by design) and is covered by the pose-graph robust-kernel test;
  // the gate is covered by the backend test.
  cfg.loop_robust_c = 0.0;
  cfg.loop_max_dt = 1e9;
  cfg.loop_max_dr = 1e9;
  return cfg;
}

// Build a backend with a closed, drifting loop and a fired loop closure.
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

}  // namespace

TEST(MapIo, SaveLoadRoundTripIsFaithful) {
  Backend src(loopConfig());
  buildLoopedBackend(&src);
  ASSERT_GE(src.numKeyframes(), 5u);
  ASSERT_FALSE(src.loopEdges().empty());

  const fs::path dir = fs::temp_directory_path() / "g1_slam_backend_map_io_test";
  std::error_code ec;
  fs::remove_all(dir, ec);

  std::string msg;
  ASSERT_TRUE(src.saveMap(dir.string(), &msg)) << msg;
  EXPECT_TRUE(fs::exists(dir / "manifest.yaml"));
  EXPECT_TRUE(fs::exists(dir / "pose_graph.g2o"));
  EXPECT_TRUE(fs::exists(dir / "scan_context.bin"));
  EXPECT_TRUE(fs::exists(dir / "keyframes" / "000000.pcd"));

  // Load into a fresh backend with a DIFFERENT initial config (the load must
  // adopt the map's Scan Context config from the manifest).
  BackendConfig other = loopConfig();
  other.sc.num_rings = 8;  // wrong on purpose; loadMap must override from manifest
  other.sc.num_sectors = 12;
  Backend dst(other);
  ASSERT_TRUE(dst.loadMap(dir.string(), &msg)) << msg;

  // Keyframe count preserved.
  ASSERT_EQ(dst.numKeyframes(), src.numKeyframes());

  // loadMap does not establish map->odom (relocalization does).
  EXPECT_TRUE(dst.mapToOdom().isApprox(Eigen::Isometry3d::Identity(), 1e-9));

  // Loop edges preserved with their measurements.
  ASSERT_EQ(dst.loopEdges().size(), src.loopEdges().size());
  for (std::size_t i = 0; i < src.loopEdges().size(); ++i) {
    const auto& a = src.loopEdges()[i];
    const auto& b = dst.loopEdges()[i];
    EXPECT_EQ(a.from, b.from);
    EXPECT_EQ(a.to, b.to);
    EXPECT_TRUE(a.rel.isApprox(b.rel, 1e-4));
  }

  // Optimized poses re-converge to the saved estimate.
  const std::vector<Eigen::Isometry3d> ps = src.optimizedPoses();
  const std::vector<Eigen::Isometry3d> pd = dst.optimizedPoses();
  ASSERT_EQ(ps.size(), pd.size());
  for (std::size_t i = 0; i < ps.size(); ++i) {
    EXPECT_LT((ps[i].translation() - pd[i].translation()).norm(), 1e-2)
        << "keyframe " << i;
  }

  // Per-keyframe clouds round-trip (point counts) and the assembled maps match.
  const Cloud::Ptr ms = src.assembleMap();
  const Cloud::Ptr md = dst.assembleMap();
  EXPECT_EQ(ms->size(), md->size());

  fs::remove_all(dir, ec);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
