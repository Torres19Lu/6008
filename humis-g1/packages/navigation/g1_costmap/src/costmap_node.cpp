// ROS node for the 2D layered costmap (ground-relative 3-state model). I/O only:
// subscribes the static SLAM map + the live registered cloud, derives the ground
// from the stance foot via TF, drives the ROS-free LayeredCostmap on a fixed-rate
// timer, and publishes /nav/costmap. The static layer (and the per-cell ground
// surface) rebuilds on each map message; the dynamic + inflation + fuse + publish
// run on the timer (the heavy point transform happens off the lock).

#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include <std_srvs/Empty.h>

#include <g1_msgs/KeyframeCloudArray.h>

#include <g1_costmap/CostmapTuningConfig.h>

#include "g1_costmap/core/cost_values.h"
#include "g1_costmap/core/foot_ground.h"
#include "g1_costmap/core/ground_classifier.h"
#include "g1_costmap/core/local_window.h"
#include "g1_costmap/layers/layered_costmap.h"

namespace g1_costmap {
namespace {

std::vector<Eigen::Vector3f> extractPoints(const sensor_msgs::PointCloud2& msg,
                                           const Eigen::Isometry3f& T,
                                           bool apply) {
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(static_cast<std::size_t>(msg.width) * msg.height);
  sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
  for (; ix != ix.end(); ++ix, ++iy, ++iz) {
    if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) continue;
    Eigen::Vector3f p(*ix, *iy, *iz);
    pts.push_back(apply ? (T * p) : p);
  }
  return pts;
}

// Build a StaticKeyframe from a map-frame keyframe cloud, optionally estimating
// per-point surface normals (KNN PCA) for the normal gate. Points and normals
// stay parallel (NaN points dropped in both); a degenerate normal is stored as
// the zero vector so the gate falls back to pure height bands for that point.
void extractKeyframe(const sensor_msgs::PointCloud2& msg, bool with_normals,
                     int normal_k, StaticKeyframe& out) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg, *cloud);
  const int k = std::max(3, normal_k);
  if (with_normals && static_cast<int>(cloud->size()) >= k) {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal> normals;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(k);
    ne.compute(normals);
    out.points.reserve(cloud->size());
    out.normals.reserve(cloud->size());
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      const pcl::PointXYZ& p = (*cloud)[i];
      if (!pcl::isFinite(p)) continue;
      out.points.emplace_back(p.x, p.y, p.z);
      const pcl::Normal& n = normals[i];
      if (std::isfinite(n.normal_x) && std::isfinite(n.normal_y) &&
          std::isfinite(n.normal_z)) {
        out.normals.emplace_back(n.normal_x, n.normal_y, n.normal_z);
      } else {
        out.normals.emplace_back(0.0f, 0.0f, 0.0f);
      }
    }
  } else {
    out.points.reserve(cloud->size());
    for (const pcl::PointXYZ& p : *cloud) {
      if (!pcl::isFinite(p)) continue;
      out.points.emplace_back(p.x, p.y, p.z);
    }
  }
}

}  // namespace

class CostmapNode {
 public:
  CostmapNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : tf_listener_(tf_buffer_) {
    std::string keyframes_topic, cloud_topic, costmap_topic, prob_topic, raw_topic,
        change_prob_topic;
    pnh.param<std::string>("keyframes_topic", keyframes_topic, "/slam/keyframes");
    pnh.param<std::string>("cloud_topic", cloud_topic,
                           "/slam/frontend/cloud_registered");
    pnh.param<std::string>("costmap_topic", costmap_topic, "/nav/costmap");
    pnh.param<std::string>("costmap_prob_topic", prob_topic, "/nav/costmap_prob");
    pnh.param<std::string>("costmap_change_prob_topic", change_prob_topic,
                           "/nav/change_prob");
    pnh.param<std::string>("costmap_raw_topic", raw_topic, "/nav/costmap_raw");
    pnh.param<std::string>("map_frame", map_frame_, "map");
    pnh.param<std::string>("lidar_frame", lidar_frame_, "mid360_link");
    pnh.param<std::string>("left_foot_frame", left_foot_frame_,
                           "left_ankle_roll_link");
    pnh.param<std::string>("right_foot_frame", right_foot_frame_,
                           "right_ankle_roll_link");
    // Foot sole = 4 collision spheres in the *_ankle_roll_link frame (identical
    // both feet, from g1_description ankle_roll collision). updateFootGround
    // projects the centres through the foot TF and subtracts the radius. Static
    // physical constants, so loaded here, not via dynamic_reconfigure.
    pnh.param("foot_sole_z", foot_sole_z_, foot_sole_z_);
    pnh.param("foot_sole_radius", foot_sole_radius_, foot_sole_radius_);
    pnh.param("foot_toe_x", foot_toe_x_, foot_toe_x_);
    pnh.param("foot_heel_x", foot_heel_x_, foot_heel_x_);
    pnh.param("foot_front_half_y", foot_front_half_y_, foot_front_half_y_);
    pnh.param("foot_rear_half_y", foot_rear_half_y_, foot_rear_half_y_);
    foot_centres_ = {
        {foot_toe_x_, foot_front_half_y_, foot_sole_z_},
        {foot_toe_x_, -foot_front_half_y_, foot_sole_z_},
        {foot_heel_x_, foot_rear_half_y_, foot_sole_z_},
        {foot_heel_x_, -foot_rear_half_y_, foot_sole_z_},
    };
    pnh.param("update_rate", update_rate_, 5.0);
    pnh.param("transform_tolerance", transform_tolerance_, 0.2);

    // Local window params (rolling robot-centered crop). Code defaults match YAML.
    pnh.param("local_costmap_enable", local_costmap_enable_, true);
    pnh.param<std::string>("local_costmap_topic", local_costmap_topic_,
                           "/nav/local_costmap");
    pnh.param("local_size_m", local_size_m_, 6.0);
    pnh.param<std::string>("robot_base_frame", robot_base_frame_, "base_link");

    // clear_costmap service params. Code defaults match YAML.
    pnh.param("clear_service_enable", clear_service_enable_, true);
    pnh.param<std::string>("clear_service_name", clear_service_name_,
                           "clear_costmap");

    // Localization-phase guard: ignore keyframe updates after the first.
    pnh.param("freeze_static", freeze_static_, false);

    // Costmap config: structural fields here, live fields refreshed by the
    // dynamic_reconfigure callback (which fires once at server construction).
    pnh.param("resolution", cfg_.resolution, cfg_.resolution);
    pnh.param("margin", cfg_.margin, cfg_.margin);
    pnh.param("xy_clip_pct", cfg_.xy_clip_pct, cfg_.xy_clip_pct);
    pnh.param("max_fill_radius", cfg_.max_fill_radius, cfg_.max_fill_radius);
    pnh.param("ground_min_points", cfg_.ground_min_points, cfg_.ground_min_points);
    pnh.param("hole_fill_iters", cfg_.hole_fill_iters, cfg_.hole_fill_iters);
    pnh.param("dyn_max_range", cfg_.dyn_max_range, cfg_.dyn_max_range);
    pnh.param("dyn_raycast", cfg_.dyn_raycast, cfg_.dyn_raycast);
    pnh.param("dyn_obstacle_frac", cfg_.dyn_obstacle_frac, cfg_.dyn_obstacle_frac);
    pnh.param("dyn_min_returns", cfg_.dyn_min_returns, cfg_.dyn_min_returns);
    pnh.param("dyn_prob_hit", cfg_.dyn_prob_hit, cfg_.dyn_prob_hit);
    pnh.param("dyn_prob_miss", cfg_.dyn_prob_miss, cfg_.dyn_prob_miss);
    pnh.param("dyn_clamp_min", cfg_.dyn_clamp_min, cfg_.dyn_clamp_min);
    pnh.param("dyn_clamp_max", cfg_.dyn_clamp_max, cfg_.dyn_clamp_max);
    pnh.param("dyn_fill_thr", cfg_.dyn_fill_thr, cfg_.dyn_fill_thr);
    pnh.param("robot_radius", cfg_.robot_radius, cfg_.robot_radius);
    // Static raycast + normal gate + log-odds accumulation (structural).
    pnh.param("static_max_range", cfg_.static_max_range, cfg_.static_max_range);
    pnh.param("use_normals", cfg_.use_normals, cfg_.use_normals);
    pnh.param("normal_k", normal_k_, normal_k_);
    pnh.param("ground_normal_angle", cfg_.ground_normal_angle,
              cfg_.ground_normal_angle);
    pnh.param("vertical_normal_angle", cfg_.vertical_normal_angle,
              cfg_.vertical_normal_angle);
    pnh.param("normal_flat_grace_layers", cfg_.normal_flat_grace_layers,
              cfg_.normal_flat_grace_layers);
    pnh.param("prob_hit", cfg_.prob_hit, cfg_.prob_hit);
    pnh.param("prob_miss", cfg_.prob_miss, cfg_.prob_miss);
    pnh.param("prob_clamp_min", cfg_.prob_clamp_min, cfg_.prob_clamp_min);
    pnh.param("prob_clamp_max", cfg_.prob_clamp_max, cfg_.prob_clamp_max);
    pnh.param("occupancy_thr", cfg_.occupancy_thr, cfg_.occupancy_thr);
    pnh.param("erode_obstacles", cfg_.erode_obstacles, cfg_.erode_obstacles);
    pnh.param("angular_fill", cfg_.angular_fill, cfg_.angular_fill);
    pnh.param("angular_fill_step_deg", cfg_.angular_fill_step_deg,
              cfg_.angular_fill_step_deg);
    // Live fields seeded from YAML too, so costmap_ is valid before the first
    // reconfigure callback.
    pnh.param("walkable_layers", cfg_.walkable_layers, cfg_.walkable_layers);
    pnh.param("obstacle_max_height", cfg_.obstacle_max_height,
              cfg_.obstacle_max_height);
    pnh.param("dyn_decay_half_life", cfg_.dyn_decay_half_life,
              cfg_.dyn_decay_half_life);
    pnh.param("dyn_add_thr", cfg_.dyn_add_thr, cfg_.dyn_add_thr);
    pnh.param("dyn_clear_thr", cfg_.dyn_clear_thr, cfg_.dyn_clear_thr);
    pnh.param("inflation_radius", cfg_.inflation_radius, cfg_.inflation_radius);
    pnh.param("cost_scaling_factor", cfg_.cost_scaling_factor,
              cfg_.cost_scaling_factor);

    costmap_ = std::make_unique<LayeredCostmap>(cfg_);

    // Inflated costmap (free/obstacle/unknown + inflation band) for the planner;
    // RViz renders it with the standard "costmap" colour scheme. No custom colour
    // cloud: /nav/costmap (costmap scheme) covers the inflation view.
    costmap_pub_ =
        nh.advertise<nav_msgs::OccupancyGrid>(costmap_topic, 1, /*latch=*/true);
    // Probabilistic occupancy grid (log-odds of the static fusion, no inflation):
    // render in RViz as a semi-transparent Map overlay to inspect fusion quality.
    prob_pub_ =
        nh.advertise<nav_msgs::OccupancyGrid>(prob_topic, 1, /*latch=*/true);
    // Dynamic change-evidence probability (no inflation): inspect / tune the
    // add/clear thresholds in RViz as a semi-transparent Map overlay.
    change_prob_pub_ =
        nh.advertise<nav_msgs::OccupancyGrid>(change_prob_topic, 1, /*latch=*/true);
    // Inflation-free 3-state occupancy (free/obstacle/unknown), the
    // 3-state grid map; the default RViz view (rviz/Map, "map" colour scheme).
    raw_pub_ =
        nh.advertise<nav_msgs::OccupancyGrid>(raw_topic, 1, /*latch=*/true);
    // Rolling robot-centered local window (inflated master crop) for the MPC
    // local planner. Only advertised when local_costmap_enable is true.
    if (local_costmap_enable_) {
      local_costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(
          local_costmap_topic_, 1, /*latch=*/true);
    }
    if (clear_service_enable_) {
      clear_srv_ = pnh.advertiseService(clear_service_name_,
                                        &CostmapNode::clearCostmapSrv, this);
      ROS_INFO("g1_costmap: clear_costmap service at %s/%s",
               pnh.getNamespace().c_str(), clear_service_name_.c_str());
    }
    kf_sub_ = nh.subscribe(keyframes_topic, 1, &CostmapNode::keyframesCb, this);
    cloud_sub_ = nh.subscribe(cloud_topic, 1, &CostmapNode::cloudCb, this);

    recfg_server_ = std::make_unique<
        dynamic_reconfigure::Server<CostmapTuningConfig>>(pnh);
    recfg_server_->setCallback(
        boost::bind(&CostmapNode::reconfigureCb, this, _1, _2));

    timer_ = nh.createTimer(ros::Duration(1.0 / update_rate_),
                            &CostmapNode::updateTimerCb, this);
    ROS_INFO("g1_costmap up: keyframes=%s cloud=%s out=%s frame=%s rate=%.1f Hz"
             " local_window=%s size=%.1fm",
             keyframes_topic.c_str(), cloud_topic.c_str(), costmap_topic.c_str(),
             map_frame_.c_str(), update_rate_,
             local_costmap_enable_ ? local_costmap_topic_.c_str() : "(disabled)",
             local_size_m_);
  }

 private:
  // Service handler for clear_costmap (std_srvs/Empty): clears the dynamic
  // (transient-obstacle) layer under the mutex and returns immediately. The next
  // timer tick re-fuses and republishes the cleared master -- do not fuse here.
  bool clearCostmapSrv(std_srvs::Empty::Request& /*req*/,
                       std_srvs::Empty::Response& /*res*/) {
    std::lock_guard<std::mutex> lk(mu_);
    costmap_->clearDynamic();
    ROS_INFO_THROTTLE(1.0, "g1_costmap: dynamic layer cleared (stale obstacles removed)");
    return true;
  }

  // Backend keyframe stream (each: sensor origin + map-frame cloud): rebuild the
  // ground surface + static layer by raycasting free space from each origin.
  void keyframesCb(const g1_msgs::KeyframeCloudArray::ConstPtr& msg) {
    std::vector<StaticKeyframe> kfs;
    kfs.reserve(msg->keyframes.size());
    std::size_t total = 0;
    for (const auto& k : msg->keyframes) {
      StaticKeyframe sk;
      sk.origin = Eigen::Vector3f(k.origin.x, k.origin.y, k.origin.z);
      extractKeyframe(k.cloud, cfg_.use_normals, normal_k_, sk);
      total += sk.points.size();
      kfs.push_back(std::move(sk));
    }
    std::lock_guard<std::mutex> lk(mu_);
    if (freeze_static_ && costmap_->hasStatic()) {
      ROS_INFO_THROTTLE(10.0,
          "g1_costmap: freeze_static set, ignoring keyframe update");
      return;
    }
    if (costmap_->setStaticKeyframes(kfs)) {
      static_ready_ = true;
      ROS_INFO(
          "static map: %zu keyframes, %zu pts -> grid %ux%u @ %.2f m, ground %zu",
          kfs.size(), total, costmap_->master().sizeX(),
          costmap_->master().sizeY(), cfg_.resolution,
          costmap_->groundSurface().cellCount());
    } else {
      ROS_WARN_THROTTLE(5.0, "keyframe message had no points");
    }
  }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mu_);
    latest_cloud_ = msg;
  }

  void reconfigureCb(CostmapTuningConfig& c, uint32_t /*level*/) {
    std::lock_guard<std::mutex> lk(mu_);
    cfg_.walkable_layers = c.walkable_layers;
    cfg_.obstacle_max_height = c.obstacle_max_height;
    cfg_.dyn_decay_half_life = c.dyn_decay_half_life;
    cfg_.dyn_add_thr = c.dyn_add_thr;
    cfg_.dyn_clear_thr = c.dyn_clear_thr;
    cfg_.dyn_obstacle_frac = c.dyn_obstacle_frac;
    cfg_.dyn_min_returns = c.dyn_min_returns;
    cfg_.inflation_radius = c.inflation_radius;
    cfg_.cost_scaling_factor = c.cost_scaling_factor;
    costmap_->setConfig(cfg_);
  }

  // Look up T(map<-source) at `stamp`; on failure fall back to `last` (if valid)
  // with a throttled warning. Returns false only when no cache exists.
  bool lookup(const std::string& source, const ros::Time& stamp,
              Eigen::Isometry3d& out, Eigen::Isometry3d& last, bool& last_valid) {
    try {
      const geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(
          map_frame_, source, stamp, ros::Duration(transform_tolerance_));
      out = tf2::transformToEigen(tf);
      last = out;
      last_valid = true;
      return true;
    } catch (const tf2::TransformException& e) {
      ROS_WARN_THROTTLE(2.0, "TF %s<-%s failed (%s); reusing last good",
                        map_frame_.c_str(), source.c_str(), e.what());
      if (last_valid) {
        out = last;
        return true;
      }
      return false;
    }
  }

  // Ground z from the feet: project each foot's sole-sphere centres through its
  // resolved TF (foot tilt exact, not just the origin z), take the lowest map Z,
  // minus the sphere radius. The min over points picks the stance (lower) foot.
  // Uses whichever ankle TFs resolve; if neither resolves this tick, leaves the
  // costmap's foot scalar unchanged.
  void updateFootGround(const ros::Time& stamp) {
    Eigen::Isometry3d lt, rt;
    const bool ok_l = lookup(left_foot_frame_, stamp, lt, last_left_, left_valid_);
    const bool ok_r =
        lookup(right_foot_frame_, stamp, rt, last_right_, right_valid_);
    if (!ok_l && !ok_r) return;
    const double ground = footGroundFromContacts(lt, ok_l, rt, ok_r,
                                                 foot_centres_, foot_sole_radius_);
    std::lock_guard<std::mutex> lk(mu_);
    costmap_->setFootGround(ground);
  }

  void updateTimerCb(const ros::TimerEvent&) {
    sensor_msgs::PointCloud2ConstPtr cloud;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!static_ready_) return;
      cloud = latest_cloud_;
    }

    // Build the dynamic scan in map frame OFF the lock (TF + point transform).
    std::vector<Eigen::Vector3f> dyn_pts;
    Eigen::Vector3f sensor_origin(0, 0, 0);
    bool have_dynamic = false;
    if (cloud) {
      const ros::Time stamp = cloud->header.stamp;
      updateFootGround(stamp);  // refresh the foot-contact ground scalar
      Eigen::Isometry3d T_map_cloud, T_map_lidar;
      const bool ok_cloud = lookup(cloud->header.frame_id, stamp, T_map_cloud,
                                   last_T_map_cloud_, last_cloud_valid_);
      // The lidar frame gives the raycast origin AND the lidar-plane band top z.
      const bool ok_lidar =
          lookup(lidar_frame_, stamp, T_map_lidar, last_T_map_lidar_,
                 last_lidar_valid_);
      if (ok_cloud && ok_lidar) {
        dyn_pts = extractPoints(*cloud, T_map_cloud.cast<float>(), true);
        sensor_origin = T_map_lidar.translation().cast<float>();
        have_dynamic = true;
      }
    }

    // Look up the robot base pose for the local window crop (OFF the lock,
    // same last-good-TF-on-miss pattern used for the other frames above).
    double base_x = 0.0, base_y = 0.0;
    bool have_base = false;
    if (local_costmap_enable_) {
      const ros::Time base_stamp =
          cloud ? cloud->header.stamp : ros::Time::now();
      Eigen::Isometry3d T_map_base;
      have_base = lookup(robot_base_frame_, base_stamp, T_map_base,
                         last_T_map_base_, last_base_valid_);
      if (have_base) {
        base_x = T_map_base.translation().x();
        base_y = T_map_base.translation().y();
      }
    }

    nav_msgs::OccupancyGrid grid;
    nav_msgs::OccupancyGrid prob_grid;
    nav_msgs::OccupancyGrid raw_grid;
    nav_msgs::OccupancyGrid local_grid;
    nav_msgs::OccupancyGrid change_grid;
    bool have_prob = false;
    bool have_local = false;
    bool have_change = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (have_dynamic) {
        costmap_->updateDynamic(dyn_pts, sensor_origin,
                                cloud->header.stamp.toSec());
      }
      const CostmapGrid& m = costmap_->fuse();
      toMsg(m, grid);                       // inflated costmap (planner)
      toMsg(costmap_->occupancy(), raw_grid);  // 3-state grid map (pre-inflation)
      const std::vector<std::int8_t>& prob = costmap_->staticProbability();
      if (prob.size() == m.cells()) {
        toProbMsg(m, prob, prob_grid);
        have_prob = true;
      }
      const std::vector<std::int8_t>& chg = costmap_->changeProbability();
      if (chg.size() == m.cells()) {
        toProbMsg(m, chg, change_grid);
        have_change = true;
      }
      // Build the local rolling window from the inflated master after fuse().
      if (local_costmap_enable_ && have_base) {
        const CostmapGrid local_crop =
            cropCentered(m, base_x, base_y, local_size_m_);
        if (local_crop.cells() > 0) {
          toMsg(local_crop, local_grid);
          have_local = true;
        }
      }
    }
    costmap_pub_.publish(grid);
    raw_pub_.publish(raw_grid);
    if (have_prob) prob_pub_.publish(prob_grid);
    if (have_change) change_prob_pub_.publish(change_grid);
    if (have_local) local_costmap_pub_.publish(local_grid);

    int nf = 0, no = 0, ni = 0, nu = 0;
    for (const std::int8_t v : grid.data) {
      if (v < 0) ++nu;
      else if (v == 0) ++nf;
      else if (v >= 100) ++no;
      else ++ni;
    }
    ROS_INFO_THROTTLE(5.0,
        "costmap %ux%u: free=%d obstacle=%d inflated=%d unknown=%d",
        grid.info.width, grid.info.height, nf, no, ni, nu);
  }

  void toMsg(const CostmapGrid& m, nav_msgs::OccupancyGrid& grid) const {
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = map_frame_;
    grid.info.resolution = m.resolution();
    grid.info.width = m.sizeX();
    grid.info.height = m.sizeY();
    grid.info.origin.position.x = m.originX();
    grid.info.origin.position.y = m.originY();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(m.cells());
    const std::vector<std::uint8_t>& cost = m.data();
    for (std::size_t i = 0; i < cost.size(); ++i) {
      grid.data[i] = costToOccupancy(cost[i]);
    }
  }

  // Probability grid: same geometry as the costmap, cells = the static log-odds
  // occupancy (-1 unknown, else 0..100). No inflation (raw fusion confidence).
  void toProbMsg(const CostmapGrid& m, const std::vector<std::int8_t>& prob,
                 nav_msgs::OccupancyGrid& grid) const {
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = map_frame_;
    grid.info.resolution = m.resolution();
    grid.info.width = m.sizeX();
    grid.info.height = m.sizeY();
    grid.info.origin.position.x = m.originX();
    grid.info.origin.position.y = m.originY();
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(prob.begin(), prob.end());
  }

  ros::Subscriber kf_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher costmap_pub_;
  ros::Publisher prob_pub_;
  ros::Publisher change_prob_pub_;  // /nav/change_prob (change-evidence diagnostic)
  ros::Publisher raw_pub_;
  ros::Publisher local_costmap_pub_;  // /nav/local_costmap (local MPC window, rolling crop)
  ros::ServiceServer clear_srv_;      // clear_costmap (dynamic-layer flush service)
  ros::Timer timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<dynamic_reconfigure::Server<CostmapTuningConfig>> recfg_server_;

  std::mutex mu_;
  std::unique_ptr<LayeredCostmap> costmap_;
  CostmapConfig cfg_;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  bool static_ready_ = false;

  std::string map_frame_, lidar_frame_, left_foot_frame_, right_foot_frame_;
  // Foot sole contact geometry (ankle_roll frame), from g1_description collision.
  double foot_sole_z_ = -0.03;        // m; sphere centre z below the ankle-roll origin
  double foot_sole_radius_ = 0.005;   // m; sphere radius (subtracted after projection)
  double foot_toe_x_ = 0.12;          // m; front (toe) contact x
  double foot_heel_x_ = -0.05;        // m; rear (heel) contact x
  double foot_front_half_y_ = 0.03;   // m; toe +/- y
  double foot_rear_half_y_ = 0.025;   // m; heel +/- y
  std::vector<Eigen::Vector3d> foot_centres_;  // built from the above at startup
  double update_rate_ = 5.0;
  double transform_tolerance_ = 0.2;
  int normal_k_ = 20;        // KNN for per-point normal estimation (keyframe gate)

  // Local window params (rolling robot-centered crop for the MPC local planner).
  bool local_costmap_enable_ = true;
  std::string local_costmap_topic_ = "/nav/local_costmap";
  double local_size_m_ = 6.0;
  std::string robot_base_frame_ = "base_link";

  // clear_costmap service params (dynamic-layer flush lever for recovery).
  bool clear_service_enable_ = true;
  std::string clear_service_name_ = "clear_costmap";

  // Localization-phase static freeze guard.
  bool freeze_static_ = false;

  // Last good transforms, reused on a TF lookup miss (graceful degradation).
  // Touched only by the timer thread (updateTimerCb / updateFootGround run on
  // the timer), never by the subscriber callbacks, so they need no extra lock.
  Eigen::Isometry3d last_T_map_cloud_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_T_map_lidar_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_T_map_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_left_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_right_ = Eigen::Isometry3d::Identity();
  bool last_cloud_valid_ = false;
  bool last_lidar_valid_ = false;
  bool last_base_valid_ = false;
  bool left_valid_ = false;
  bool right_valid_ = false;
};

}  // namespace g1_costmap

int main(int argc, char** argv) {
  ros::init(argc, argv, "g1_costmap");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  g1_costmap::CostmapNode node(nh, pnh);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
