// Backend end-to-end unit test.
//
// Drive a closed square loop with accumulating odometry drift past a fixed,
// asymmetric landmark field. The robot returns to its start pose, so the final
// keyframe's surroundings match the first keyframe's: Scan Context detects the
// revisit, ICP verifies it, and iSAM2 corrects the drift. Assert that a loop
// fires between the start and end keyframes and that map->odom pulls the end
// back near truth.

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "g1_slam_backend/backend.h"

using g1_slam_backend::Backend;
using g1_slam_backend::BackendConfig;

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

Eigen::Isometry3d xyPose(double x, double y) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << x, y, 0.0;
  return T;
}

// Fixed world landmarks (asymmetric so each true pose has a distinct view):
// {world x, world y, column height}.
const std::vector<Eigen::Vector3d>& landmarks() {
  static const std::vector<Eigen::Vector3d> lm = {
      {3, 2, 1.0},  {7, -1, 2.0}, {-4, 5, 1.5}, {10, 3, 0.8},
      {-6, -3, 2.5}, {5, 8, 1.2}, {-8, 1, 1.8}, {2, -7, 2.2}};
  return lm;
}

// The full landmark field as vertical columns, in the world frame, shifted by
// (sx, sy). (Shifting by the odom drift yields the registered odom-frame cloud.)
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

// Base config for the closed-square-loop scenario (the new loop-robustness knobs
// are left at their defaults; individual tests override them).
BackendConfig baseLoopCfg() {
  BackendConfig cfg;
  cfg.keyframe_dist = 1.0;
  cfg.keyframe_angle = 0.5;
  cfg.keyframe_voxel = 0.0;  // keep synthetic clouds exact for clean ICP
  cfg.map_voxel = 0.0;
  cfg.sc_min_id_gap = 3;
  cfg.sc_dist_thresh = 0.2;
  cfg.sc_knn = 5;
  cfg.icp_max_corr_dist = 3.0;  // must exceed the accumulated drift
  cfg.icp_max_iter = 50;
  cfg.icp_fitness_thresh = 0.5;
  cfg.icp_submap_neighbors = 0;
  cfg.loop_icp_voxel = 0.0;  // keep the exact synthetic landmark columns for clean ICP
  cfg.loop_rot_sigma = 0.02;
  cfg.loop_trans_sigma = 0.05;
  cfg.loop_extra_iters = 10;
  return cfg;
}

struct LoopRun {
  std::size_t nkf = 0;
  bool any_loop = false;
  bool has_0_8 = false;   // a loop edge between start (0) and end (8) keyframes
  double corrected_err = 0.0;  // optimized end-keyframe distance from truth (0,0)
  double map_to_odom = 0.0;
};

// Drive a closed square loop (last pose == first pose == truth (0,0)) with
// accumulating odometry drift past an asymmetric landmark field, then drain the
// loop-closure queue. The registered odom-frame cloud = world field shifted by
// the drift, so back in base_link the drift cancels and the true geometry remains.
LoopRun driveSquareLoop(const BackendConfig& cfg, double drift_per_step) {
  const std::vector<Eigen::Vector2d> truth = {
      {0, 0}, {2, 0}, {4, 0}, {4, 2}, {4, 4}, {2, 4}, {0, 4}, {0, 2}, {0, 0}};
  Backend backend(cfg);
  for (std::size_t i = 0; i < truth.size(); ++i) {
    const double d = drift_per_step * static_cast<double>(i);
    const Eigen::Isometry3d odom = xyPose(truth[i].x() + d, truth[i].y() + d);
    const Cloud::Ptr cw = worldCloud(d, d);
    backend.stepIntake(static_cast<double>(i), odom, *cw);
  }
  LoopRun r;
  r.nkf = backend.numKeyframes();
  for (int i = 0; i < 50 && backend.hasPendingLoopChecks(); ++i) {
    r.any_loop = backend.runLoopClosureOnce() || r.any_loop;
  }
  for (const auto& e : backend.loopEdges()) {
    if (std::min(e.from, e.to) == 0u && std::max(e.from, e.to) == 8u) r.has_0_8 = true;
  }
  r.corrected_err = backend.optimizedPoses().back().translation().head<2>().norm();
  r.map_to_odom = backend.mapToOdom().translation().norm();
  return r;
}

}  // namespace

TEST(Backend, ClosedLoopCorrectsDriftViaScanContextAndIcp) {
  // Isolate the correction pipeline (SC -> ICP -> loop -> iSAM2) from the new loop
  // robustness: a plain Gaussian factor (no Cauchy) and the consistency gate
  // disabled, so this single true loop fully corrects the (large, synthetic) drift.
  BackendConfig cfg = baseLoopCfg();
  cfg.loop_robust_c = 0.0;   // plain factor -> full correction (robust kernel tested elsewhere)
  cfg.loop_max_dt = 1e9;     // disable the consistency gate (tested separately below)
  cfg.loop_max_dr = 1e9;

  const LoopRun r = driveSquareLoop(cfg, 0.08);
  ASSERT_EQ(r.nkf, 9u);
  EXPECT_TRUE(r.any_loop);
  EXPECT_TRUE(r.has_0_8);
  EXPECT_GT(Eigen::Vector2d(8 * 0.08, 8 * 0.08).norm(), 0.5);  // raw drift is large
  EXPECT_LT(r.corrected_err, 0.3);   // closure pulled the end back near truth
  EXPECT_GT(r.map_to_odom, 0.5);     // non-trivial map->odom correction
}

TEST(Backend, ConsistencyGateRejectsLoopInconsistentWithOdometry) {
  // Same revisit, but with the default front-end consistency gate active. The
  // injected drift (~0.9 m over an ~8 m path) makes the ICP loop measurement
  // disagree with the odometry-chained relative beyond the drift budget, so the
  // loop is rejected before it can distort the graph (an accurate front end
  // treats such a disagreement as a false/degenerate match).
  // The contrast with ClosedLoop above (identical scenario, gate disabled, which
  // DOES form the 0-8 loop) isolates the gate: the robust kernel only down-weights
  // (it still adds the edge), so has_0_8 == false can come ONLY from the gate.
  BackendConfig cfg = baseLoopCfg();  // gate + Cauchy kernel at defaults
  const LoopRun r = driveSquareLoop(cfg, 0.08);
  ASSERT_EQ(r.nkf, 9u);
  EXPECT_FALSE(r.has_0_8);          // the odom-inconsistent 0-8 loop is gated out
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
