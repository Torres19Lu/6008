// GTSAM + PCL + Eigen ABI smoke test.
//
// This is intentionally the FIRST build artifact of g1_slam_backend. The real
// risk with GTSAM is not "which install" but an ABI/Eigen mismatch: GTSAM is
// often built with -march=native and its own Eigen alignment assumptions, and
// when linked alongside PCL/ROS-Eigen built differently it segfaults at runtime
// (not compile time). Exercising GTSAM and PCL in one binary here forces such a
// mismatch to surface immediately, before any backend logic is written.

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// PCL + Eigen in the same translation unit as GTSAM: a voxel filter must run.
TEST(GtsamAbiSmoke, PclVoxelFilterRuns) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < 1000; ++i) {
    cloud->push_back(pcl::PointXYZ(0.001f * i, 0.001f * i, 0.0f));
  }
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.05f, 0.05f, 0.05f);
  pcl::PointCloud<pcl::PointXYZ> out;
  vg.filter(out);
  EXPECT_GT(cloud->size(), out.size());
  EXPECT_GT(out.size(), 0u);
}

// iSAM2: prior on x0 at the origin, a between x0->x1 of +1 m in x; optimize and
// read x1 back. Confirms GTSAM links, runs, and returns sane numbers - the
// exact iSAM2 path the pose-graph core (Task 2) will use.
TEST(GtsamAbiSmoke, Isam2OptimizesBetweenFactor) {
  using gtsam::Pose3;
  using gtsam::Point3;
  using gtsam::Rot3;

  gtsam::ISAM2 isam;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;

  const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
  const auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());

  const Pose3 origin;
  const Pose3 step(Rot3(), Point3(1.0, 0.0, 0.0));

  graph.emplace_shared<gtsam::PriorFactor<Pose3>>(0, origin, prior_noise);
  graph.emplace_shared<gtsam::BetweenFactor<Pose3>>(0, 1, step, odom_noise);
  values.insert(0, origin);
  values.insert(1, origin);  // deliberately wrong init; the optimizer must move it

  isam.update(graph, values);
  isam.update();
  const gtsam::Values result = isam.calculateEstimate();

  ASSERT_TRUE(result.exists(1));
  const Pose3 x1 = result.at<Pose3>(1);
  EXPECT_NEAR(x1.x(), 1.0, 1e-3);
  EXPECT_NEAR(x1.y(), 0.0, 1e-3);
  EXPECT_NEAR(x1.z(), 0.0, 1e-3);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
