// Node-level rostest for the map-management backend hooks (Tasks 4/5/6 node side):
//   - read-only guard: save_map is refused after load_map(read_only=true), and no
//     map file changes (non-destructive localization).
//   - /slam/keyframe_poses: latched full keyframe poses are published on load.
//   - /slam/begin_incremental guard: refused unless localized on a loaded map.
//   - /slam/reset: succeeds.
//
// The fixture map is built IN-PROCESS with the controllable core (no synthetic SLAM
// publishing over ROS), then the running node's services are exercised. The
// relocalize -> locked_pose -> append happy path needs a live matching scan and is
// covered by the core gtest (re-base math) + the owner-in-the-loop gate, consistent
// with the package's bag-gated estimator policy.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <g1_msgs/KeyframePoseArray.h>
#include <g1_msgs/LoadMap.h>
#include <g1_msgs/SaveMap.h>
#include <std_srvs/Trigger.h>

#include "g1_slam_backend/backend.h"

using g1_slam_backend::Backend;
using g1_slam_backend::BackendConfig;

namespace {

namespace fs = std::filesystem;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

Eigen::Isometry3d xyYawPose(double x, double y, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << x, y, 0.0;
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T;
}

Cloud::Ptr worldCloud(double sx, double sy) {
  static const std::vector<Eigen::Vector3d> lm = {
      {3, 2, 1.0},   {7, -1, 2.0}, {-4, 5, 1.5}, {10, 3, 0.8},
      {-6, -3, 2.5}, {5, 8, 1.2},  {-8, 1, 1.8}, {2, -7, 2.2}};
  Cloud::Ptr c(new Cloud());
  for (const Eigen::Vector3d& l : lm) {
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
  cfg.icp_submap_neighbors = 0;
  cfg.loop_icp_voxel = 0.0;
  cfg.loop_robust_c = 0.0;
  cfg.loop_max_dt = 1e9;
  cfg.loop_max_dr = 1e9;
  return cfg;
}

// A drifting square loop with per-keyframe yaw, so the saved keyframes have
// non-identity orientations (needed for the keyframe_poses orientation check).
void buildLoopedBackend(Backend* backend) {
  const std::vector<Eigen::Vector3d> truth = {  // x, y, yaw
      {0, 0, 0.0},  {2, 0, 0.0},  {4, 0, 1.57}, {4, 2, 1.57}, {4, 4, 3.14},
      {2, 4, 3.14}, {0, 4, -1.57}, {0, 2, -1.57}, {0, 0, 0.0}};
  const double drift = 0.08;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    const double d = drift * static_cast<double>(i);
    backend->stepIntake(static_cast<double>(i),
                        xyYawPose(truth[i].x() + d, truth[i].y() + d, truth[i].z()),
                        *worldCloud(d, d));
  }
  for (int i = 0; i < 50 && backend->hasPendingLoopChecks(); ++i) {
    backend->runLoopClosureOnce();
  }
}

// Read every regular file under dir into relpath -> contents, so a test can assert
// byte-for-byte that nothing changed.
std::map<std::string, std::string> snapshotDir(const fs::path& dir) {
  std::map<std::string, std::string> out;
  for (const auto& e : fs::recursive_directory_iterator(dir)) {
    if (!e.is_regular_file()) continue;
    std::ifstream is(e.path(), std::ios::binary);
    std::ostringstream ss;
    ss << is.rdbuf();
    out[fs::relative(e.path(), dir).string()] = ss.str();
  }
  return out;
}

}  // namespace

TEST(BackendNodeHooks, ReadOnlyGuardKeyframePosesAndIncrementalGuard) {
  ros::NodeHandle nh;

  // Build a fixture map in-process and persist it.
  Backend src(loopConfig());
  buildLoopedBackend(&src);
  ASSERT_GE(src.numKeyframes(), 5u);
  const fs::path dir = fs::temp_directory_path() / "g1_slam_backend_node_hooks_map";
  std::error_code ec;
  fs::remove_all(dir, ec);
  std::string msg;
  ASSERT_TRUE(src.saveMap(dir.string(), &msg)) << msg;

  auto load = nh.serviceClient<g1_msgs::LoadMap>("/slam/load_map");
  auto save = nh.serviceClient<g1_msgs::SaveMap>("/slam/save_map");
  auto begin_incr = nh.serviceClient<std_srvs::Trigger>("/slam/begin_incremental");
  auto reset = nh.serviceClient<std_srvs::Trigger>("/slam/reset");
  ASSERT_TRUE(load.waitForExistence(ros::Duration(20.0))) << "node services not up";

  // Load the map read-only (localization).
  g1_msgs::LoadMap lm;
  lm.request.path = dir.string();
  lm.request.read_only = true;
  ASSERT_TRUE(load.call(lm));
  ASSERT_TRUE(lm.response.success) << lm.response.message;

  // Task 5: /slam/keyframe_poses is latched and carries one full pose per keyframe,
  // with a non-identity orientation somewhere (the loop turns).
  auto poses = ros::topic::waitForMessage<g1_msgs::KeyframePoseArray>(
      "/slam/keyframe_poses", nh, ros::Duration(10.0));
  ASSERT_TRUE(poses != nullptr) << "no /slam/keyframe_poses received";
  EXPECT_EQ(poses->poses.size(), src.numKeyframes());
  bool any_rotated = false;
  for (const auto& kp : poses->poses) {
    if (std::abs(kp.pose.orientation.w) < 0.999) any_rotated = true;
  }
  EXPECT_TRUE(any_rotated) << "expected a rotated keyframe orientation";

  // Task 4: save_map is refused while read-only, and nothing on disk changes.
  const std::map<std::string, std::string> before = snapshotDir(dir);
  const fs::path forbidden = fs::temp_directory_path() / "g1_slam_backend_should_not_write";
  fs::remove_all(forbidden, ec);
  g1_msgs::SaveMap sm;
  sm.request.path = forbidden.string();
  ASSERT_TRUE(save.call(sm));
  EXPECT_FALSE(sm.response.success);
  EXPECT_NE(sm.response.message.find("read-only"), std::string::npos)
      << "message was: " << sm.response.message;
  EXPECT_FALSE(fs::exists(forbidden)) << "read-only save must not write any directory";
  EXPECT_EQ(snapshotDir(dir), before) << "read-only load must leave the map untouched";

  // Task 6: begin_incremental is refused unless localized on a loaded map.
  std_srvs::Trigger bt;
  ASSERT_TRUE(begin_incr.call(bt));
  EXPECT_FALSE(bt.response.success) << "begin_incremental must require a lock first";

  // Task 6: reset returns the node to a fresh mapping session.
  std_srvs::Trigger rt;
  ASSERT_TRUE(reset.call(rt));
  EXPECT_TRUE(rt.response.success);

  // After reset (mapping, read_only cleared), save_map is allowed again.
  const fs::path allowed = fs::temp_directory_path() / "g1_slam_backend_after_reset";
  fs::remove_all(allowed, ec);
  g1_msgs::SaveMap sm2;
  sm2.request.path = allowed.string();
  ASSERT_TRUE(save.call(sm2));
  // reset cleared the keyframes, so saveMap reports "no keyframes to save" but is NOT
  // refused for being read-only -> the guard is cleared.
  EXPECT_EQ(sm2.response.message.find("read-only"), std::string::npos)
      << "read-only guard should be cleared after reset; message: " << sm2.response.message;

  fs::remove_all(dir, ec);
  fs::remove_all(allowed, ec);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "backend_node_hooks_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  const int rc = RUN_ALL_TESTS();
  ros::shutdown();
  return rc;
}
