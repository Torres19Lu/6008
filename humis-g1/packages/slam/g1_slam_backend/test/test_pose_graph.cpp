// Pose-graph core unit test.
//
// Planted-loop drift correction: walk a closed square with a constant per-edge
// translation drift so the open-loop end lands well off truth, then add the true
// loop-closure edge and assert iSAM2 pulls the end back and redistributes the
// correction around the loop while the anchored first node stays put.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "g1_slam_backend/pose_graph.h"

using g1_slam_backend::PoseGraph;

namespace {

Eigen::Isometry3d xlate(double x, double y, double z = 0.0) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << x, y, z;
  return T;
}

double posDist(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return (a.translation() - b.translation()).norm();
}

}  // namespace

TEST(PoseGraph, PlantedLoopCorrectsDrift) {
  // True positions around a 2 m square, 8 nodes, identity rotation.
  const std::vector<Eigen::Isometry3d> truth = {
      xlate(0, 0), xlate(1, 0), xlate(2, 0), xlate(2, 1),
      xlate(2, 2), xlate(1, 2), xlate(0, 2), xlate(0, 1)};
  const int N = static_cast<int>(truth.size());

  const Eigen::Isometry3d drift = xlate(0.05, 0.05);  // per-edge translation error

  PoseGraph pg;
  pg.addPrior(0, truth[0], 1e-4, 1e-4);
  for (int i = 0; i + 1 < N; ++i) {
    const Eigen::Isometry3d true_rel = truth[i].inverse() * truth[i + 1];
    const Eigen::Isometry3d measured = true_rel * drift;  // inject drift
    pg.addOdometry(i, i + 1, measured, 0.05, 0.1);
  }
  pg.update();

  // Open-loop: all nodes present, and the last node has drifted off truth.
  EXPECT_EQ(pg.size(), static_cast<std::size_t>(N));
  EXPECT_TRUE(pg.exists(0));
  const double open_gap = posDist(pg.optimizedPose(N - 1), truth[N - 1]);
  EXPECT_GT(open_gap, 0.25);
  const Eigen::Isometry3d mid_open = pg.optimizedPose(4);

  // Loop closure: the true relative from the last node back to node 0 (as ICP
  // would report it), drift-free.
  const Eigen::Isometry3d loop_rel = truth[N - 1].inverse() * truth[0];
  pg.addLoop(N - 1, 0, loop_rel, 0.02, 0.05, 0.0);  // plain: a true loop
  pg.update(10);

  // Closed-loop: the end is pulled back near truth, the anchored node 0 stays
  // put, and an interior node has moved from its open-loop estimate.
  const double closed_gap = posDist(pg.optimizedPose(N - 1), truth[N - 1]);
  EXPECT_LT(closed_gap, 0.15);
  EXPECT_LT(open_gap - closed_gap, open_gap);  // closure strictly improved the end
  EXPECT_LT(posDist(pg.optimizedPose(0), truth[0]), 1e-2);
  EXPECT_GT(posDist(pg.optimizedPose(4), mid_open), 0.02);
}

TEST(PoseGraph, RobustKernelRejectsOutlierLoop) {
  // Drift-free odometry around the same square (open loop == truth), then inject
  // ONE gross FALSE loop claiming node 4 (2,2) coincides with node 0 (0,0). A
  // plain Gaussian factor drags the graph off truth; the Cauchy robust kernel
  // down-weights the outlier so the graph stays near truth. This is the map
  // distortion (and its fix) reproduced at the pose-graph layer.
  const std::vector<Eigen::Isometry3d> truth = {
      xlate(0, 0), xlate(1, 0), xlate(2, 0), xlate(2, 1),
      xlate(2, 2), xlate(1, 2), xlate(0, 2), xlate(0, 1)};
  const int N = static_cast<int>(truth.size());
  const Eigen::Isometry3d false_loop = Eigen::Isometry3d::Identity();  // "4 == 0"

  auto build = [&](double cauchy_c) {
    auto pg = std::make_unique<PoseGraph>();
    pg->addPrior(0, truth[0], 1e-4, 1e-4);
    for (int i = 0; i + 1 < N; ++i) {
      pg->addOdometry(i, i + 1, truth[i].inverse() * truth[i + 1], 0.05, 0.1);
    }
    pg->update();
    pg->addLoop(4, 0, false_loop, 0.3, 0.5, cauchy_c);  // gross outlier
    pg->update(10);
    double worst = 0.0;
    for (int i = 0; i < N; ++i) {
      worst = std::max(worst, posDist(pg->optimizedPose(i), truth[i]));
    }
    return worst;
  };

  const double plain = build(0.0);   // no robust kernel
  const double robust = build(1.0);  // Cauchy
  EXPECT_GT(plain, 0.3);             // the false loop wrecks the plain graph
  EXPECT_LT(robust, 0.1);            // Cauchy keeps it near truth
  EXPECT_LT(robust, 0.5 * plain);    // and is a large, clear improvement
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
