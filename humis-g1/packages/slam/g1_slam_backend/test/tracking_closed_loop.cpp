// Node-level closed-loop rostest: the robust tracking correction must NOT break
// NORMAL localization. In a well-conditioned scene (a landmark field that
// constrains x/y/yaw) with a slowly DRIFTING front-end odom, the running node
// must keep map->odom tracking the true (drifting) offset, so the robot's map
// pose stays put. This is the regression guard for the localization-tracking
// robustness change (projection/clamp/low-pass/jump-reject): if any layer were
// too aggressive it would lag or coast and this test would fail.
//
// The robot is stationary at a known map pose X_true; the odom frame drifts by a
// known M(t) each tick. We publish the matching (odom_pose, registered_cloud),
// lock once via /slam/relocalize, then stream the drift and assert the node's
// broadcast map->odom converges to the final true offset (and actually moved off
// the initial one, i.e. it tracked rather than coasted).

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <mutex>
#include <vector>

#include <ros/ros.h>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#include <g1_msgs/LoadMap.h>
#include <g1_msgs/Relocalize.h>

#include "g1_slam_backend/backend.h"

using g1_slam_backend::Backend;
using g1_slam_backend::BackendConfig;

namespace {

namespace fs = std::filesystem;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

Eigen::Isometry3d pose(double x, double y, double yaw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() << x, y, 0.0;
  return T;
}

geometry_msgs::Pose eigenToPose(const Eigen::Isometry3d& T) {
  geometry_msgs::Pose p;
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();
  const Eigen::Quaterniond q(T.rotation());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

// Asymmetric vertical-column field (world == map): constrains x, y and yaw.
Cloud::Ptr worldField() {
  static const std::vector<Eigen::Vector3d> lm = {
      {3, 2, 1.0},   {7, -1, 2.0}, {-4, 5, 1.5}, {10, 3, 0.8},
      {-6, -3, 2.5}, {5, 8, 1.2},  {-8, 1, 1.8}, {2, -7, 2.2}};
  Cloud::Ptr c(new Cloud());
  for (const Eigen::Vector3d& l : lm) {
    for (double z = 0.0; z <= l.z() + 1e-9; z += 0.2) {
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

BackendConfig mapConfig() {
  BackendConfig cfg;
  cfg.keyframe_dist = 1.0;
  cfg.keyframe_angle = 0.5;
  cfg.keyframe_voxel = 0.0;  // exact synthetic clouds
  cfg.map_voxel = 0.0;
  cfg.icp_submap_neighbors = 0;
  return cfg;
}

std::vector<Eigen::Isometry3d> buildMap(Backend* backend) {
  const std::vector<Eigen::Isometry3d> truth = {
      pose(0, 0, 0.0), pose(2, 0, 0.0), pose(4, 0, 0.0),
      pose(4, 2, 0.0), pose(4, 4, 0.0), pose(2, 4, 0.0)};
  const Cloud::Ptr field = worldField();
  for (std::size_t i = 0; i < truth.size(); ++i) {
    backend->stepIntake(static_cast<double>(i), truth[i], *field);
  }
  return truth;
}

// Latest map->odom captured off /tf.
struct TfGrab {
  std::mutex mu;
  bool have = false;
  Eigen::Isometry3d m2o = Eigen::Isometry3d::Identity();
  void cb(const tf2_msgs::TFMessage::ConstPtr& msg) {
    for (const auto& t : msg->transforms) {
      if (t.header.frame_id == "map" && t.child_frame_id == "odom") {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        const auto& q = t.transform.rotation;
        const auto& v = t.transform.translation;
        T.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
        T.translation() << v.x, v.y, v.z;
        std::lock_guard<std::mutex> lk(mu);
        m2o = T;
        have = true;
      }
    }
  }
};

void publishFrame(ros::Publisher& odom_pub, ros::Publisher& cloud_pub,
                  const Eigen::Isometry3d& m2o, const Eigen::Isometry3d& x_true) {
  const Eigen::Isometry3d p_now = m2o.inverse() * x_true;  // sensor odom pose
  const Cloud::Ptr live = transformCloud(m2o.inverse(), *worldField());
  const ros::Time stamp = ros::Time::now();

  nav_msgs::Odometry od;
  od.header.stamp = stamp;
  od.header.frame_id = "odom";
  od.child_frame_id = "base_link";
  od.pose.pose = eigenToPose(p_now);
  odom_pub.publish(od);

  sensor_msgs::PointCloud2 cl;
  pcl::toROSMsg(*live, cl);
  cl.header.stamp = stamp;
  cl.header.frame_id = "odom";
  cloud_pub.publish(cl);
}

void expectClose(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b,
                 double trans_tol, double rot_tol, const char* what) {
  EXPECT_LT((a.translation() - b.translation()).norm(), trans_tol) << what;
  const Eigen::AngleAxisd d(a.rotation().transpose() * b.rotation());
  EXPECT_LT(std::abs(d.angle()), rot_tol) << what;
}

}  // namespace

TEST(TrackingClosedLoop, AbsorbsOdomDriftInWellConditionedScene) {
  ros::NodeHandle nh;

  // Build + persist a fixture map, then have the node load it read-only.
  Backend src(mapConfig());
  const std::vector<Eigen::Isometry3d> truth = buildMap(&src);
  ASSERT_EQ(src.numKeyframes(), truth.size());
  const fs::path dir = fs::temp_directory_path() / "g1_slam_backend_tracking_loop_map";
  std::error_code ec;
  fs::remove_all(dir, ec);
  std::string msg;
  ASSERT_TRUE(src.saveMap(dir.string(), &msg)) << msg;
  const Eigen::Isometry3d x_true = src.optimizedPoses()[2];  // robot sits at kf 2

  auto load = nh.serviceClient<g1_msgs::LoadMap>("/slam/load_map");
  auto reloc = nh.serviceClient<g1_msgs::Relocalize>("/slam/relocalize");
  ASSERT_TRUE(load.waitForExistence(ros::Duration(20.0))) << "node services not up";

  ros::Publisher odom_pub =
      nh.advertise<nav_msgs::Odometry>("/slam/frontend/odom", 10);
  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/slam/frontend/cloud_registered", 10);
  TfGrab tf;
  ros::Subscriber tf_sub = nh.subscribe("/tf", 50, &TfGrab::cb, &tf);

  g1_msgs::LoadMap lm;
  lm.request.path = dir.string();
  lm.request.read_only = true;
  ASSERT_TRUE(load.call(lm));
  ASSERT_TRUE(lm.response.success) << lm.response.message;

  // Initial true offset; stream a few frames so the sync caches a live scan.
  const Eigen::Isometry3d m0 = pose(0.30, -0.20, 0.05);
  for (int i = 0; i < 10 && ros::ok(); ++i) {
    publishFrame(odom_pub, cloud_pub, m0, x_true);
    ros::Duration(0.05).sleep();
  }

  // Lock with a guided relocalization at the true map pose.
  g1_msgs::Relocalize rq;
  rq.request.use_guess = true;
  rq.request.initial_guess = eigenToPose(x_true);
  ASSERT_TRUE(reloc.call(rq));
  ASSERT_TRUE(rq.response.success) << "guided relocalization failed to lock";

  // Stream a slow odom drift (x, y and yaw) the tracker must absorb.
  Eigen::Isometry3d m_final = m0;
  const int N = 60;
  for (int t = 1; t <= N && ros::ok(); ++t) {
    m_final = pose(0.30 + 0.0040 * t, -0.20 + 0.0020 * t, 0.05 + 0.0008 * t);
    publishFrame(odom_pub, cloud_pub, m_final, x_true);
    ros::Duration(0.03).sleep();
  }
  // Hold the final frame so the tracker settles and re-broadcasts.
  for (int i = 0; i < 30 && ros::ok(); ++i) {
    publishFrame(odom_pub, cloud_pub, m_final, x_true);
    ros::Duration(0.03).sleep();
  }

  Eigen::Isometry3d m2o;
  bool have;
  {
    std::lock_guard<std::mutex> lk(tf.mu);
    have = tf.have;
    m2o = tf.m2o;
  }
  ASSERT_TRUE(have) << "node never broadcast map->odom (never localized/tracked)";

  // Total imposed drift is ~0.27 m / 0.05 rad: well above tolerance, so a tracker
  // that coasted (stuck near m0) would fail the convergence check below.
  const double drift = (m_final.translation() - m0.translation()).norm();
  ASSERT_GT(drift, 0.20);

  // 1) The node tracked the drift: map->odom matches the FINAL true offset.
  expectClose(m2o, m_final, 0.10, 0.05, "map->odom should track the drifting offset");
  // 2) It actually moved (did not coast on odom): closer to m_final than to m0.
  EXPECT_LT((m2o.translation() - m_final.translation()).norm(),
            (m2o.translation() - m0.translation()).norm())
      << "map->odom coasted instead of tracking";
  // 3) Equivalently, the robot's map pose stayed put at x_true.
  const Eigen::Isometry3d robot_map = m2o * (m_final.inverse() * x_true);
  expectClose(robot_map, x_true, 0.10, 0.05, "robot map pose should stay at x_true");

  fs::remove_all(dir, ec);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tracking_closed_loop_test");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  const int rc = RUN_ALL_TESTS();
  ros::shutdown();
  return rc;
}
