// ROS wiring for the SLAM backend. All logic lives in Backend (ROS-free); this
// file only does I/O: it syncs the front-end odom + registered cloud into the
// fast stepIntake() path, runs the slow runLoopClosureOnce() on a separate timer
// under a mutex, broadcasts map->odom, and publishes the latched map + pose graph.

#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_srvs/Trigger.h>

#include <g1_msgs/KeyframeCloud.h>
#include <g1_msgs/KeyframeCloudArray.h>
#include <g1_msgs/KeyframePose.h>
#include <g1_msgs/KeyframePoseArray.h>
#include <g1_msgs/LoadMap.h>
#include <g1_msgs/Relocalize.h>
#include <g1_msgs/SaveMap.h>

#include "reloc_quality.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Geometry>

#include "g1_slam_backend/backend.h"
#include "g1_slam_backend/tracking_correction.h"

namespace g1_slam_backend {
namespace {

Eigen::Isometry3d poseToEigen(const geometry_msgs::Pose& p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y,
                             p.orientation.z);
  t.linear() = q.normalized().toRotationMatrix();
  return t;
}

geometry_msgs::Pose eigenToPose(const Eigen::Isometry3d& T) {
  geometry_msgs::Pose p;
  const Eigen::Vector3d tr = T.translation();
  p.position.x = tr.x();
  p.position.y = tr.y();
  p.position.z = tr.z();
  const Eigen::Quaterniond q(T.rotation());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

// Build a line/point marker set for the pose graph: nodes, odometry edges, and
// loop edges. Caller holds the backend mutex.
visualization_msgs::MarkerArray buildGraphMarkers(const Backend& backend,
                                                  const std::string& frame,
                                                  const ros::Time& stamp) {
  visualization_msgs::MarkerArray arr;
  const std::vector<Eigen::Isometry3d> poses = backend.optimizedPoses();

  auto base = [&](int id, int type, double scale, float r, float g, float b) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = "pose_graph";
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = scale;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;
    m.pose.orientation.w = 1.0;
    return m;
  };
  auto toPt = [](const Eigen::Isometry3d& p) {
    geometry_msgs::Point pt;
    pt.x = p.translation().x();
    pt.y = p.translation().y();
    pt.z = p.translation().z();
    return pt;
  };

  visualization_msgs::Marker nodes =
      base(0, visualization_msgs::Marker::SPHERE_LIST, 0.2, 0.1f, 0.9f, 0.1f);
  for (const auto& p : poses) nodes.points.push_back(toPt(p));
  arr.markers.push_back(nodes);

  visualization_msgs::Marker odom_edges =
      base(1, visualization_msgs::Marker::LINE_LIST, 0.04, 0.2f, 0.4f, 1.0f);
  for (std::size_t i = 1; i < poses.size(); ++i) {
    odom_edges.points.push_back(toPt(poses[i - 1]));
    odom_edges.points.push_back(toPt(poses[i]));
  }
  arr.markers.push_back(odom_edges);

  visualization_msgs::Marker loop_edges =
      base(2, visualization_msgs::Marker::LINE_LIST, 0.06, 1.0f, 0.1f, 0.1f);
  for (const auto& e : backend.loopEdges()) {
    if (e.from < poses.size() && e.to < poses.size()) {
      loop_edges.points.push_back(toPt(poses[e.from]));
      loop_edges.points.push_back(toPt(poses[e.to]));
    }
  }
  arr.markers.push_back(loop_edges);
  return arr;
}

}  // namespace

class BackendNode {
 public:
  BackendNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    BackendConfig cfg;
    std::string odom_topic, cloud_topic, map_topic, graph_topic, kf_topic,
        kf_poses_topic;
    int sync_queue = 100;
    double sync_slop = 0.05, tf_rate = 20.0, loop_rate = 1.0;

    pnh.param<std::string>("odom_topic", odom_topic, "/slam/frontend/odom");
    pnh.param<std::string>("cloud_topic", cloud_topic,
                           "/slam/frontend/cloud_registered");
    pnh.param<std::string>("map_topic", map_topic, "/slam/map");
    pnh.param<std::string>("pose_graph_topic", graph_topic, "/slam/pose_graph");
    pnh.param<std::string>("keyframes_topic", kf_topic, "/slam/keyframes");
    pnh.param<std::string>("keyframe_poses_topic", kf_poses_topic,
                           "/slam/keyframe_poses");
    pnh.param<std::string>("map_frame", map_frame_, "map");
    pnh.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh.param("transform_tolerance", transform_tolerance_, 0.1);
    pnh.param("tf_publish_rate", tf_rate, 20.0);
    pnh.param("loop_closure_rate", loop_rate, 1.0);
    pnh.param("sync_queue", sync_queue, 100);
    pnh.param("sync_slop", sync_slop, 0.05);

    pnh.param("keyframe_dist", cfg.keyframe_dist, cfg.keyframe_dist);
    pnh.param("keyframe_angle", cfg.keyframe_angle, cfg.keyframe_angle);
    pnh.param("keyframe_voxel", cfg.keyframe_voxel, cfg.keyframe_voxel);
    pnh.param("map_voxel", cfg.map_voxel, cfg.map_voxel);
    pnh.param("sc/num_rings", cfg.sc.num_rings, cfg.sc.num_rings);
    pnh.param("sc/num_sectors", cfg.sc.num_sectors, cfg.sc.num_sectors);
    pnh.param("sc/max_range", cfg.sc.max_range, cfg.sc.max_range);
    pnh.param("sc/min_range", cfg.sc.min_range, cfg.sc.min_range);
    pnh.param("sc/height_offset", cfg.sc.height_offset, cfg.sc.height_offset);
    pnh.param("sc_min_id_gap", cfg.sc_min_id_gap, cfg.sc_min_id_gap);
    pnh.param("sc_dist_thresh", cfg.sc_dist_thresh, cfg.sc_dist_thresh);
    pnh.param("sc_knn", cfg.sc_knn, cfg.sc_knn);
    pnh.param("icp/max_corr_dist", cfg.icp_max_corr_dist, cfg.icp_max_corr_dist);
    pnh.param("icp/max_iter", cfg.icp_max_iter, cfg.icp_max_iter);
    pnh.param("icp/fitness_thresh", cfg.icp_fitness_thresh, cfg.icp_fitness_thresh);
    pnh.param("icp/submap_neighbors", cfg.icp_submap_neighbors,
              cfg.icp_submap_neighbors);
    pnh.param("icp/loop_voxel", cfg.loop_icp_voxel, cfg.loop_icp_voxel);
    pnh.param("noise/prior_rot", cfg.prior_rot_sigma, cfg.prior_rot_sigma);
    pnh.param("noise/prior_trans", cfg.prior_trans_sigma, cfg.prior_trans_sigma);
    pnh.param("noise/odom_rot", cfg.odom_rot_sigma, cfg.odom_rot_sigma);
    pnh.param("noise/odom_trans", cfg.odom_trans_sigma, cfg.odom_trans_sigma);
    pnh.param("noise/loop_rot", cfg.loop_rot_sigma, cfg.loop_rot_sigma);
    pnh.param("noise/loop_trans", cfg.loop_trans_sigma, cfg.loop_trans_sigma);
    pnh.param("loop_extra_iters", cfg.loop_extra_iters, cfg.loop_extra_iters);

    // Loop-closure robustness (Cauchy kernel + front-end consistency gate): keeps
    // false/degenerate loops (corridors, repetitive rooms) from distorting the map.
    pnh.param("loop/robust_c", cfg.loop_robust_c, cfg.loop_robust_c);
    pnh.param("loop/max_dt", cfg.loop_max_dt, cfg.loop_max_dt);
    pnh.param("loop/max_dt_rate", cfg.loop_max_dt_rate, cfg.loop_max_dt_rate);
    pnh.param("loop/max_dr", cfg.loop_max_dr, cfg.loop_max_dr);
    pnh.param("loop/max_dr_rate", cfg.loop_max_dr_rate, cfg.loop_max_dr_rate);

    // Relocalization geometric acceptance (inlier ratio, not fitness alone).
    pnh.param("reloc/min_inlier", cfg.reloc_min_inlier, cfg.reloc_min_inlier);
    pnh.param("reloc/inlier_dist", cfg.reloc_inlier_dist, cfg.reloc_inlier_dist);
    pnh.param("reloc/num_candidates", cfg.reloc_num_candidates,
              cfg.reloc_num_candidates);

    // Relocalization timing / search (node-level).
    pnh.param("reloc/search_radius", reloc_search_radius_, reloc_search_radius_);
    pnh.param("reloc/lost_streak", reloc_lost_streak_, reloc_lost_streak_);
    pnh.param("reloc/acquire_stride", acquire_stride_, acquire_stride_);
    if (acquire_stride_ < 1) acquire_stride_ = 1;
    pnh.param("reloc/rate", reloc_rate_, reloc_rate_);
    if (reloc_rate_ <= 0.0) reloc_rate_ = 5.0;
    pnh.param("reloc/track_rebuild_dist", track_rebuild_dist_, track_rebuild_dist_);
    pnh.param("track_normal_k", cfg.track_normal_k, cfg.track_normal_k);

    // Localization tracking robustness: instead of overwriting map->odom with the
    // raw per-tick ICP result, the node projects the correction off unobservable
    // DOFs (degeneracy), clamps and low-passes it, and rejects (coasts on the
    // smooth front-end odom) an implausibly large jump. Defaults disable nothing
    // that matters; tune on a localization bag.
    pnh.param("reloc/obs_eig_floor", track_corr_.obs_eig_floor, track_corr_.obs_eig_floor);
    pnh.param("reloc/obs_eig_ratio", track_corr_.obs_eig_ratio, track_corr_.obs_eig_ratio);
    pnh.param("reloc/corr_gain", track_corr_.corr_gain, track_corr_.corr_gain);
    pnh.param("reloc/max_step_trans", track_corr_.max_step_trans, track_corr_.max_step_trans);
    pnh.param("reloc/max_step_rot", track_corr_.max_step_rot, track_corr_.max_step_rot);
    pnh.param("reloc/reject_trans", track_corr_.reject_trans, track_corr_.reject_trans);
    pnh.param("reloc/reject_rot", track_corr_.reject_rot, track_corr_.reject_rot);

    std::string save_srv, load_srv, reloc_srv, reset_srv, begin_incr_srv, map_path;
    pnh.param<std::string>("save_map_service", save_srv, "/slam/save_map");
    pnh.param<std::string>("load_map_service", load_srv, "/slam/load_map");
    pnh.param<std::string>("relocalize_service", reloc_srv, "/slam/relocalize");
    pnh.param<std::string>("reset_service", reset_srv, "/slam/reset");
    pnh.param<std::string>("begin_incremental_service", begin_incr_srv,
                           "/slam/begin_incremental");
    pnh.param<std::string>("map_path", map_path, "");

    backend_ = std::make_unique<Backend>(cfg);

    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(map_topic, 1, /*latch=*/true);
    graph_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>(graph_topic, 1, /*latch=*/true);
    // Per-keyframe clouds + sensor origins (map frame) for the costmap to raycast
    // free space; latched, republished on optimization/load like /slam/map.
    kf_pub_ =
        nh.advertise<g1_msgs::KeyframeCloudArray>(kf_topic, 1, /*latch=*/true);
    // Lightweight full keyframe poses (no clouds) for nav-point anchoring; latched,
    // republished on intake/optimization/load alongside /slam/keyframes.
    kf_poses_pub_ =
        nh.advertise<g1_msgs::KeyframePoseArray>(kf_poses_topic, 1, /*latch=*/true);

    odom_sub_.subscribe(nh, odom_topic, sync_queue);
    cloud_sub_.subscribe(nh, cloud_topic, sync_queue);
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(sync_queue), odom_sub_, cloud_sub_);
    sync_->setMaxIntervalDuration(ros::Duration(sync_slop));
    sync_->registerCallback(
        boost::bind(&BackendNode::syncCb, this, _1, _2));

    loop_timer_ = nh.createTimer(ros::Duration(1.0 / loop_rate),
                                 &BackendNode::loopTimerCb, this);
    tf_timer_ = nh.createTimer(ros::Duration(1.0 / tf_rate),
                               &BackendNode::tfTimerCb, this);
    reloc_timer_ = nh.createTimer(ros::Duration(1.0 / reloc_rate_),
                                  &BackendNode::relocTimerCb, this);

    save_srv_ = nh.advertiseService(save_srv, &BackendNode::saveMapSrv, this);
    load_srv_ = nh.advertiseService(load_srv, &BackendNode::loadMapSrv, this);
    reloc_srv_ = nh.advertiseService(reloc_srv, &BackendNode::relocalizeSrv, this);
    reset_srv_ = nh.advertiseService(reset_srv, &BackendNode::resetSrv, this);
    begin_incr_srv_ =
        nh.advertiseService(begin_incr_srv, &BackendNode::beginIncrementalSrv, this);

    // Optionally load a map at startup and begin in localization mode.
    if (!map_path.empty()) {
      std::string msg;
      if (backend_->loadMap(map_path, &msg)) {
        mode_ = Mode::LOCALIZING;
        localized_ = false;
        have_submap_ = false;
        acquire_counter_ = 0;
        ROS_INFO_STREAM("g1_slam_backend: loaded map from " << map_path << " ("
                        << msg << "); localization mode, awaiting relocalize");
        publishOutputs(ros::Time::now());
      } else {
        ROS_ERROR_STREAM("g1_slam_backend: load map from " << map_path
                         << " failed: " << msg);
      }
    }

    ROS_INFO("g1_slam_backend: up. odom=%s cloud=%s map_frame=%s odom_frame=%s mode=%s",
             odom_topic.c_str(), cloud_topic.c_str(), map_frame_.c_str(),
             odom_frame_.c_str(), mode_ == Mode::MAPPING ? "mapping" : "localizing");
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      nav_msgs::Odometry, sensor_msgs::PointCloud2>;

  void syncCb(const nav_msgs::Odometry::ConstPtr& odom,
              const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    const Eigen::Isometry3d odom_pose = poseToEigen(odom->pose.pose);
    Backend::Cloud::Ptr cloud(new Backend::Cloud());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    const ros::Time stamp = odom->header.stamp;

    Mode mode;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mode = mode_;
      last_odom_pose_ = odom_pose;  // cached for the relocalize service
      last_cloud_ = cloud;
      have_live_ = true;
    }

    if (mode == Mode::MAPPING) {
      bool is_kf = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        is_kf = backend_->stepIntake(stamp.toSec(), odom_pose, *cloud);
      }
      if (is_kf) publishOutputs(stamp);
    }
    // LOCALIZING is handled by relocTimerCb on its own timer (decoupled from the
    // scan rate), so the heavy reloc work does not run on every scan.
  }

  // Localization step on a fixed timer (reloc_rate), decoupled from the scan
  // rate. ACQUIRING: throttled global Scan Context until a lock.
  // TRACKING: local scan-to-map ICP around the predicted pose to absorb odom
  // drift, with the submap cached and rebuilt only when the robot moves
  // (track_rebuild_dist). The loaded map is frozen during localization, so the
  // submap assembly and the tracking ICP run OFF the mutex; only the latest
  // scan, map->odom, and the lock flags are touched under the lock. This keeps
  // the 20 Hz map->odom broadcast from being starved by the ICP.
  void relocTimerCb(const ros::TimerEvent&) {
    Eigen::Isometry3d odom_pose, pred = Eigen::Isometry3d::Identity();
    Backend::Cloud::ConstPtr cloud, submap;
    bool localized = false, do_acquire = false, rebuild = false;
    std::vector<Backend::KeyframeView> near;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ != Mode::LOCALIZING || !have_live_ || !last_cloud_) return;
      odom_pose = last_odom_pose_;
      cloud = last_cloud_;
      localized = localized_;
      if (!localized_) {
        do_acquire = (acquire_counter_++ % acquire_stride_ == 0);
      } else {
        pred = backend_->mapToOdom() * odom_pose;
        if (!have_submap_ ||
            (pred.translation() - track_center_).norm() > track_rebuild_dist_) {
          near = backend_->snapshotKeyframesNear(pred.translation(),
                                                 reloc_search_radius_);
          rebuild = true;
        } else {
          submap = track_submap_;  // reuse the cached submap
        }
      }
    }

    // ACQUIRING: global Scan Context under the lock (throttled; map->odom is not
    // broadcast while unlocked, so holding the lock here does not stall tf).
    if (!localized) {
      if (!do_acquire) return;
      Backend::RelocResult r;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        r = backend_->relocalizeGlobal(odom_pose, *cloud);
        if (r.found) {
          backend_->setMapToOdom(r.map_to_odom);
          localized_ = true;
          lost_streak_ = 0;
          have_submap_ = false;  // force a fresh tracking submap at the new pose
        }
      }
      if (r.found) {
        ROS_INFO("g1_slam_backend: relocalized (kf %lu, sc %.3f, inlier %.2f, "
                 "fitness %.3f)", static_cast<unsigned long>(r.match_id),
                 r.sc_distance, r.inlier_ratio, r.fitness);
      } else {
        ROS_WARN_THROTTLE(2.0,
            "g1_slam_backend: acquire rejected (best kf %lu, sc %.3f, inlier "
            "%.2f, fitness %.3f)", static_cast<unsigned long>(r.match_id),
            r.sc_distance, r.inlier_ratio, r.fitness);
      }
      return;
    }

    // TRACKING: assemble (if needed) and ICP OFF the lock.
    Backend::Cloud::Ptr fresh;
    if (rebuild) fresh = Backend::assembleFromSnapshot(near, /*map_voxel=*/0.0);
    const Backend::Cloud::ConstPtr target =
        rebuild ? Backend::Cloud::ConstPtr(fresh) : submap;
    if (!target || target->empty()) return;
    const Backend::RelocResult r =
        backend_->trackAgainstSubmap(odom_pose, *cloud, pred, target);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (rebuild) {
        track_submap_ = fresh;
        track_center_ = pred.translation();
        have_submap_ = true;
      }
      if (r.found) {
        // Do NOT overwrite map->odom with the raw ICP pose: near a dominant plane
        // (a table/monitor) that pose slides. Project off the unobservable DOFs,
        // clamp + low-pass the step, and reject (coast on the smooth front-end
        // odom) an implausibly large jump. The current correction is the filter
        // state; odom_pose is the live pose paired with this scan.
        const Eigen::Isometry3d x_now_raw = r.map_to_odom * odom_pose;
        const TrackingCorrectionResult rc = computeTrackingCorrection(
            backend_->mapToOdom(), odom_pose, x_now_raw, r.obs_info, track_corr_);
        if (rc.applied) {
          backend_->setMapToOdom(rc.map_to_odom);
          lost_streak_ = 0;  // a good registration (degenerate DOFs just held)
          if (rc.num_degenerate > 0) {
            ROS_INFO_THROTTLE(2.0, "g1_slam_backend: tracking holding odom on %d "
                              "unobservable DOF(s) (scene degenerate)",
                              rc.num_degenerate);
          }
        } else if (++lost_streak_ >= reloc_lost_streak_) {
          // Sustained rejects: more likely genuine confusion than a glitch.
          localized_ = false;
          lost_streak_ = 0;
          have_submap_ = false;
          ROS_WARN("g1_slam_backend: lost tracking (corrections rejected), "
                   "re-acquiring (global SC)");
        } else {
          ROS_WARN_THROTTLE(2.0, "g1_slam_backend: tracking correction rejected "
                            "(jump %.2f m); coasting on odom",
                            (x_now_raw.translation() -
                             (backend_->mapToOdom() * odom_pose).translation())
                                .norm());
        }
      } else if (++lost_streak_ >= reloc_lost_streak_) {
        localized_ = false;
        lost_streak_ = 0;
        have_submap_ = false;
        ROS_WARN("g1_slam_backend: lost tracking, re-acquiring (global SC)");
      }
    }
  }

  // Slow loop-closure pass on its own timer. Holding the mutex serializes with
  // intake; ICP latency briefly delays (but does not drop) intake callbacks. A
  // snapshot/commit split could shrink the critical section if ICP cost grows.
  void loopTimerCb(const ros::TimerEvent&) {
    bool closed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // No loop closure in localization mode (the map is frozen).
      if (mode_ == Mode::MAPPING) closed = backend_->runLoopClosureOnce();
    }
    if (closed) publishOutputs(ros::Time::now());
  }

  void tfTimerCb(const ros::TimerEvent&) {
    Eigen::Isometry3d m2o;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // In localization mode, map->odom is unknown until the first lock, so do
      // not broadcast it (the live odom-frame data floats until relocalized).
      if (mode_ == Mode::LOCALIZING && !localized_) return;
      m2o = backend_->mapToOdom();
    }
    // now()-stamp (+ tolerance) keeps the slowly-updated parent uniquely stamped
    // and fresh, avoiding TF_REPEATED_DATA (the base_footprint lesson). Guard
    // against a non-advancing stamp too: under sim time (bag replay with --clock)
    // now() moves in coarse steps, so two 20 Hz fires can share a stamp; skip the
    // duplicate. On wall clock now() always advances, so this never skips.
    const ros::Time stamp = ros::Time::now() + ros::Duration(transform_tolerance_);
    if (stamp <= last_tf_stamp_) return;
    last_tf_stamp_ = stamp;
    geometry_msgs::TransformStamped ts = tf2::eigenToTransform(m2o);
    ts.header.stamp = stamp;
    ts.header.frame_id = map_frame_;
    ts.child_frame_id = odom_frame_;
    tf_broadcaster_.sendTransform(ts);
  }

  void publishOutputs(const ros::Time& stamp) {
    // Snapshot under the lock (cheap: poses + cloud ptr refcount bumps), then do
    // the heavy transform/concat/voxel OFF the lock so it never stalls intake.
    std::vector<Backend::KeyframeView> snap;
    visualization_msgs::MarkerArray markers;
    double map_voxel = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snap = backend_->snapshotKeyframes();
      markers = buildGraphMarkers(*backend_, map_frame_, stamp);
      map_voxel = backend_->mapVoxel();
    }
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*Backend::assembleFromSnapshot(snap, map_voxel), map_msg);
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = stamp;
    map_pub_.publish(map_msg);
    graph_pub_.publish(markers);

    // Per-keyframe map-frame clouds + sensor origins for the costmap raycast.
    // Latched + the full set republished each time (like /slam/map), so
    // loop-closure re-optimization (every keyframe pose moves) and late-joining
    // subscribers both see the current optimized set.
    g1_msgs::KeyframeCloudArray arr;
    arr.header.frame_id = map_frame_;
    arr.header.stamp = stamp;
    arr.keyframes.reserve(snap.size());
    Backend::Cloud kf_map;
    for (std::size_t i = 0; i < snap.size(); ++i) {
      g1_msgs::KeyframeCloud kf;
      kf.id = i;
      const Eigen::Vector3d o = snap[i].pose.translation();
      kf.origin.x = o.x();
      kf.origin.y = o.y();
      kf.origin.z = o.z();
      kf_map.clear();
      pcl::transformPointCloud(*snap[i].cloud, kf_map,
                               snap[i].pose.matrix().cast<float>());
      pcl::toROSMsg(kf_map, kf.cloud);
      kf.cloud.header.frame_id = map_frame_;
      kf.cloud.header.stamp = stamp;
      arr.keyframes.push_back(std::move(kf));
    }
    kf_pub_.publish(arr);

    // Lightweight full keyframe poses (same ids, no clouds) for nav-point anchoring.
    g1_msgs::KeyframePoseArray poses;
    poses.header.frame_id = map_frame_;
    poses.header.stamp = stamp;
    poses.poses.reserve(snap.size());
    for (std::size_t i = 0; i < snap.size(); ++i) {
      g1_msgs::KeyframePose kp;
      kp.id = static_cast<std::uint32_t>(i);
      kp.pose = eigenToPose(snap[i].pose);
      poses.poses.push_back(std::move(kp));
    }
    kf_poses_pub_.publish(poses);
  }

  bool saveMapSrv(g1_msgs::SaveMap::Request& req,
                  g1_msgs::SaveMap::Response& res) {
    std::string msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Read-only guard: a map loaded for localization must never be overwritten.
      // This is the sole disk-write path, so the guard makes localization
      // non-destructive by construction.
      if (read_only_) {
        res.success = false;
        res.message = "map is read-only (loaded for localization); save refused";
        ROS_WARN_STREAM("g1_slam_backend: save_map(" << req.path
                        << ") refused: " << res.message);
        return true;
      }
      res.success = backend_->saveMap(req.path, &msg);
    }
    res.message = msg;
    ROS_INFO_STREAM("g1_slam_backend: save_map(" << req.path << "): " << msg);
    return true;
  }

  bool loadMapSrv(g1_msgs::LoadMap::Request& req,
                  g1_msgs::LoadMap::Response& res) {
    std::string msg;
    bool ok = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ok = backend_->loadMap(req.path, &msg);
      if (ok) {
        mode_ = Mode::LOCALIZING;  // frozen map; localize against it
        read_only_ = req.read_only;  // refuse disk writes while a read-only map is loaded
        localized_ = false;
        lost_streak_ = 0;
        acquire_counter_ = 0;
        have_submap_ = false;
      }
    }
    if (ok) publishOutputs(ros::Time::now());  // show the loaded /slam/map
    res.success = ok;
    res.message = msg;
    ROS_INFO_STREAM("g1_slam_backend: load_map(" << req.path << "): " << msg);
    return true;
  }

  bool relocalizeSrv(g1_msgs::Relocalize::Request& req,
                     g1_msgs::Relocalize::Response& res) {
    Backend::RelocResult r;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ != Mode::LOCALIZING) {
        res.success = false;
        res.message = "no map loaded (mapping mode)";
        return true;
      }
      if (!have_live_ || !last_cloud_) {
        res.success = false;
        res.message = "no live scan received yet";
        return true;
      }
      // With a guess: guided scan-to-map ICP at the guess (map frame). Without:
      // global Scan Context acquire.
      if (req.use_guess) {
        r = backend_->relocalizeLocal(last_odom_pose_, *last_cloud_,
                                      poseToEigen(req.initial_guess),
                                      reloc_search_radius_);
      } else {
        r = backend_->relocalizeGlobal(last_odom_pose_, *last_cloud_);
      }
      if (r.found) {
        backend_->setMapToOdom(r.map_to_odom);
        localized_ = true;
        lost_streak_ = 0;
        have_submap_ = false;  // rebuild the tracking submap at the new pose
        // Locked robot map pose = map->odom * P_now (live odom pose at the lock).
        res.locked_pose = eigenToPose(r.map_to_odom * last_odom_pose_);
      }
    }
    res.success = r.found;
    res.message = r.found ? "relocalized" : "relocalization failed";
    res.quality = toRelocQualityMsg(r);   // diagnostics, filled even on failure
    ROS_INFO_STREAM("g1_slam_backend: relocalize(use_guess=" << int(req.use_guess)
                    << "): " << res.message);
    return true;
  }

  bool resetSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      backend_->reset();
      mode_ = Mode::MAPPING;
      read_only_ = false;
      localized_ = false;
      lost_streak_ = 0;
      acquire_counter_ = 0;
      have_submap_ = false;
    }
    res.success = true;
    res.message = "backend reset to a fresh mapping session";
    ROS_INFO_STREAM("g1_slam_backend: " << res.message);
    return true;
  }

  // Resume MAPPING on a loaded map after a relocalization lock: re-base the loaded
  // keyframe odom poses into the live odom frame so the next keyframe's bridge
  // factor is correct. Requires LOCALIZING + a current lock; clears read_only_.
  bool beginIncrementalSrv(std_srvs::Trigger::Request&,
                           std_srvs::Trigger::Response& res) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ != Mode::LOCALIZING || !localized_) {
        res.success = false;
        res.message = "not localized on a loaded map; relocalize before incremental";
        ROS_WARN_STREAM("g1_slam_backend: begin_incremental refused: " << res.message);
        return true;
      }
      backend_->beginIncremental();
      mode_ = Mode::MAPPING;
      read_only_ = false;
    }
    res.success = true;
    res.message = "incremental mapping resumed (keyframes re-based)";
    ROS_INFO_STREAM("g1_slam_backend: " << res.message);
    return true;
  }

  enum class Mode { MAPPING, LOCALIZING };

  std::mutex mutex_;
  std::unique_ptr<Backend> backend_;
  std::string map_frame_, odom_frame_;
  double transform_tolerance_ = 0.1;
  ros::Time last_tf_stamp_;  // last map->odom stamp; guards against non-advancing stamps

  // Localization state, all guarded by mutex_.
  Mode mode_ = Mode::MAPPING;
  bool localized_ = false;        // LOCALIZING: is map->odom locked?
  int lost_streak_ = 0;           // consecutive failed tracking attempts
  long acquire_counter_ = 0;      // throttles global SC during ACQUIRING
  double reloc_search_radius_ = 20.0;
  int reloc_lost_streak_ = 5;
  int acquire_stride_ = 1;
  double reloc_rate_ = 5.0;            // Hz; localization step (off the scan path)
  double track_rebuild_dist_ = 2.0;    // m; rebuild the tracking submap past this
  // Tracking robustness (active defaults; the YAML overrides). Projection floor on
  // the mean-normalized point-to-plane info (translation eigenvalues in [0,1]),
  // gain low-pass, per-tick step clamp, and raw-jump reject (coast on odom).
  TrackingCorrectionConfig track_corr_ = [] {
    TrackingCorrectionConfig c;
    c.obs_eig_floor = 0.05;
    c.obs_eig_ratio = 0.0;
    c.corr_gain = 0.4;
    c.max_step_trans = 0.10;
    c.max_step_rot = 0.05;
    c.reject_trans = 0.50;
    c.reject_rot = 0.20;
    return c;
  }();
  Eigen::Isometry3d last_odom_pose_ = Eigen::Isometry3d::Identity();
  Backend::Cloud::Ptr last_cloud_;
  bool have_live_ = false;
  // Cached tracking submap (map frame); owned by relocTimerCb, reset under lock.
  Backend::Cloud::Ptr track_submap_;
  Eigen::Vector3d track_center_ = Eigen::Vector3d::Zero();
  bool have_submap_ = false;

  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  bool read_only_ = false;  // LOCALIZING via load_map(read_only): refuse disk writes
  ros::Publisher map_pub_, graph_pub_, kf_pub_, kf_poses_pub_;
  ros::ServiceServer save_srv_, load_srv_, reloc_srv_, reset_srv_, begin_incr_srv_;
  ros::Timer loop_timer_, tf_timer_, reloc_timer_;
};

}  // namespace g1_slam_backend

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_slam_backend");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  g1_slam_backend::BackendNode node(nh, pnh);
  // Threads so the sync callback, the loop/reloc steps, and the tf broadcast can
  // run concurrently. The heavy reloc/map work runs off the backend mutex, so
  // the tf timer is not starved.
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
