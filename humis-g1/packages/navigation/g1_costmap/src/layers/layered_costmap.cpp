#include "g1_costmap/layers/layered_costmap.h"

#include <algorithm>
#include <cmath>

#include "g1_costmap/core/occupancy_accumulator.h"  // logOdds, probFromLogOdds

namespace g1_costmap {

StaticLayerParams LayeredCostmap::staticParams() const {
  constexpr double kDegToRad = M_PI / 180.0;
  StaticLayerParams sp;
  sp.resolution = cfg_.resolution;
  sp.margin = cfg_.margin;
  sp.walkable_layers = cfg_.walkable_layers;
  sp.obstacle_max_layers = obstacleMaxLayers();
  sp.hole_fill_iters = cfg_.hole_fill_iters;
  sp.xy_clip_pct = cfg_.xy_clip_pct;
  sp.max_range = cfg_.static_max_range;
  sp.phantom_drop = cfg_.phantom_drop;
  sp.use_normals = cfg_.use_normals;
  sp.ground_normal_angle = cfg_.ground_normal_angle * kDegToRad;
  sp.vertical_normal_angle = cfg_.vertical_normal_angle * kDegToRad;
  sp.normal_flat_grace_layers = cfg_.normal_flat_grace_layers;
  sp.occ.prob_hit = cfg_.prob_hit;
  sp.occ.prob_miss = cfg_.prob_miss;
  sp.occ.prob_clamp_min = cfg_.prob_clamp_min;
  sp.occ.prob_clamp_max = cfg_.prob_clamp_max;
  sp.occ.occupancy_thr = cfg_.occupancy_thr;
  sp.erode_obstacles = cfg_.erode_obstacles;
  sp.angular_fill = cfg_.angular_fill;
  sp.angular_fill_step = cfg_.angular_fill_step_deg * kDegToRad;
  return sp;
}

bool LayeredCostmap::setStaticCloud(
    const std::vector<Eigen::Vector3f>& points_map) {
  if (points_map.empty()) return false;

  // Rebuild the per-cell ground surface (lowest-per-cell) from the map.
  surface_.configure(cfg_.resolution, cfg_.max_fill_radius,
                     cfg_.ground_min_points);
  surface_.update(points_map);

  CostmapGrid grid;
  if (!static_layer_.build(points_map, groundLookup(), staticParams(), grid))
    return false;
  static_grid_ = std::move(grid);
  static_prob_.clear();  // the bare projection carries no probability
  dynamic_layer_.configure(static_grid_);  // adopt the geometry, clear state
  return true;
}

bool LayeredCostmap::setStaticKeyframes(const std::vector<StaticKeyframe>& kfs) {
  // Ground surface from all keyframe points (lowest-per-cell, robust median).
  std::size_t n = 0;
  for (const auto& kf : kfs) n += kf.points.size();
  if (n == 0) return false;
  std::vector<Eigen::Vector3f> all;
  all.reserve(n);
  for (const auto& kf : kfs) {
    for (const auto& pt : kf.points) all.push_back(pt);
  }
  surface_.configure(cfg_.resolution, cfg_.max_fill_radius, cfg_.ground_min_points);
  surface_.update(all);

  CostmapGrid grid;
  std::vector<std::int8_t> prob;
  if (!static_layer_.buildFromKeyframes(kfs, groundLookup(), staticParams(), grid,
                                        &prob))
    return false;
  static_grid_ = std::move(grid);
  static_prob_ = std::move(prob);
  dynamic_layer_.configure(static_grid_);
  return true;
}

void LayeredCostmap::clearDynamic() {
  dynamic_layer_.clear();  // transient obstacles gone; static grid unchanged
}

void LayeredCostmap::updateDynamic(
    const std::vector<Eigen::Vector3f>& points_map,
    const Eigen::Vector3f& sensor_origin, double now) {
  if (!static_grid_.initialized()) return;
  DynamicLayerParams dp;
  dp.resolution = cfg_.resolution;
  dp.walkable_layers = cfg_.walkable_layers;
  dp.obstacle_max_layers = obstacleMaxLayers();
  dp.phantom_drop = cfg_.phantom_drop;
  dp.lidar_z = sensor_origin.z();  // the node passes the lidar-frame origin
  dp.have_lidar_z = true;
  dp.max_range = cfg_.dyn_max_range;
  dp.raycast = cfg_.dyn_raycast;
  dp.obstacle_frac = cfg_.dyn_obstacle_frac;
  dp.min_returns = cfg_.dyn_min_returns;
  dp.prob_hit = cfg_.dyn_prob_hit;
  dp.prob_miss = cfg_.dyn_prob_miss;
  dp.clamp_min = cfg_.dyn_clamp_min;
  dp.clamp_max = cfg_.dyn_clamp_max;
  dp.decay_half_life = cfg_.dyn_decay_half_life;
  dynamic_layer_.update(points_map, sensor_origin, groundLookup(), dp, now);
}

const CostmapGrid& LayeredCostmap::fuse() {
  if (!static_grid_.initialized()) return master_;
  occupancy_ = static_grid_;  // UNKNOWN / FREE / OBSTACLE from the SLAM map

  if (dynamic_layer_.initialized()) {
    const std::vector<float>& l = dynamic_layer_.logodds();
    std::vector<std::uint8_t>& cells = occupancy_.data();
    const float l_add = static_cast<float>(logOdds(cfg_.dyn_add_thr));
    const float l_fill = static_cast<float>(logOdds(cfg_.dyn_fill_thr));
    const float l_clear = static_cast<float>(logOdds(cfg_.dyn_clear_thr));
    for (std::size_t i = 0; i < cells.size(); ++i) {
      const float li = l[i];
      if (li >= l_add) {
        cells[i] = LETHAL_OBSTACLE;            // add obstacle (low bar), any static
      } else if (li <= -l_clear && cells[i] == LETHAL_OBSTACLE) {
        cells[i] = FREE_SPACE;                 // clear static LETHAL (door open; HIGH)
      } else if (li <= -l_fill && cells[i] == NO_INFORMATION) {
        cells[i] = FREE_SPACE;                 // fill static UNKNOWN (low bar)
      }
    }
    buildChangeProb();
  } else {
    change_prob_.clear();
  }

  // master_ is the inflated costmap (planner); occupancy_ stays the 3-state grid
  // map. The InflationLayer cost>current guard leaves UNKNOWN (255) and LETHAL
  // (254) untouched, so unknown space stays unknown.
  master_ = occupancy_;
  InflationParams ip;
  ip.robot_radius = cfg_.robot_radius;
  ip.inflation_radius = cfg_.inflation_radius;
  ip.cost_scaling_factor = cfg_.cost_scaling_factor;
  inflation_layer_.inflate(master_, ip);
  return master_;
}

void LayeredCostmap::buildChangeProb() {
  const std::vector<float>& l = dynamic_layer_.logodds();
  change_prob_.assign(l.size(), -1);
  for (std::size_t i = 0; i < l.size(); ++i) {
    if (l[i] == 0.0f) continue;  // neutral -> untouched (-1)
    const double pr = probFromLogOdds(static_cast<double>(l[i]));
    change_prob_[i] = static_cast<std::int8_t>(std::lround(pr * 100.0));
  }
}

}  // namespace g1_costmap
