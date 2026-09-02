#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "g1_costmap/core/costmap_grid.h"
#include "g1_costmap/layers/dynamic_layer.h"
#include "g1_costmap/core/ground_surface.h"
#include "g1_costmap/layers/inflation_layer.h"
#include "g1_costmap/layers/static_layer.h"

namespace g1_costmap {

// All costmap parameters with code defaults; the ROS node overrides them from
// config/costmap.yaml (structural) and dynamic_reconfigure (live knobs).
struct CostmapConfig {
  double resolution = 0.1;          // m / cell
  double margin = 1.0;              // m padding around the static bounds
  double xy_clip_pct = 0.005;       // robust XY grid bounds (drop this fraction each side)
  // Ground-relative band (offset = layerOf(z) - layerOf(ground_z)). The band TOP
  // is the lidar plane (sensor z - phantom_drop); obstacle_max_height is a fixed
  // SAFETY cap above ground (kept generous so the lidar plane normally bites).
  int walkable_layers = 1;          // |offset| <= this around ground -> FREE
  double obstacle_max_height = 2.0; // m above ground; safety cap on the band top
  double phantom_drop = 0.1;        // m below the lidar plane; above it -> ignored
  // Ground surface (per-cell). The foot-contact ground scalar is the node's
  // doing (sole spheres via TF, pushed in by setFootGround), not a config field.
  double max_fill_radius = 0.3;     // m; GroundSurface robust-median neighbourhood
  int ground_min_points = 1;        // returns before a cell's ground is trusted
                                    // (1: the voxel map has ~1 floor return/cell)
  int hole_fill_iters = 2;          // morphological close passes (0 disables)
  // Static raycast.
  double static_max_range = 0.0;    // m; static beam range cap (0 = unlimited)
  // Normal gate (refines the height band; degrees, converted to rad internally).
  bool use_normals = true;
  double ground_normal_angle = 40.0;   // deg; <= this is flat -> ground
  double vertical_normal_angle = 60.0; // deg; >= this is vertical -> obstacle
  int normal_flat_grace_layers = 2;    // flat surfaces up to walkable+this demote
  // Probabilistic (log-odds) accumulation across keyframes.
  double prob_hit = 0.7;
  double prob_miss = 0.4;
  double prob_clamp_min = 0.12;
  double prob_clamp_max = 0.97;
  double occupancy_thr = 0.5;
  bool erode_obstacles = true;         // remove isolated obstacle speckle
  bool angular_fill = true;            // fill unobserved wedges in front of surfaces
  double angular_fill_step_deg = 0.5;  // angular bin width for the fill
  // Dynamic layer = bidirectional log-odds change evidence vs the static map.
  double dyn_max_range = 15.0;
  bool dyn_raycast = true;
  // In-band-fraction support test (see DynamicLayer): mark a cell OBSTACLE only
  // when n_band/n_total > dyn_obstacle_frac and n_total >= dyn_min_returns.
  // Suppresses the false ground ring from live-cloud Z scatter on flat ground.
  double dyn_obstacle_frac = 0.4;    // in-band fraction to mark a cell obstacle
  int dyn_min_returns = 3;           // returns needed in a cell to apply the test
  double dyn_prob_hit = 0.7;         // P(occ | obstacle return); > 0.5
  double dyn_prob_miss = 0.4;        // P(occ | free/miss observation); < 0.5
  double dyn_clamp_min = 0.12;       // free saturation (lower clamp)
  double dyn_clamp_max = 0.97;       // obstacle saturation (upper clamp)
  double dyn_decay_half_life = 3.0;  // s; change evidence relaxes toward neutral
  double dyn_add_thr = 0.65;         // P(occ) at/above which a live obstacle is added
  double dyn_fill_thr = 0.65;        // P(free) bar to fill a static UNKNOWN
  double dyn_clear_thr = 0.85;       // P(free) bar to CLEAR a static LETHAL (high)
  // Inflation layer.
  double robot_radius = 0.3;
  double inflation_radius = 0.45;   // thin halo past the inscribed radius (keeps free visible)
  double cost_scaling_factor = 3.0;
};

// Owns the per-cell GroundSurface, the foot-contact scalar fallback, the three
// layers, and the fused master grid. The static layer fixes the grid geometry
// and builds the ground surface; the dynamic layer tracks it; fuse() combines
// them (OBSTACLE > FREE > UNKNOWN) and inflates into the published grid.
class LayeredCostmap {
 public:
  explicit LayeredCostmap(const CostmapConfig& cfg) { setConfig(cfg); }

  const CostmapConfig& config() const { return cfg_; }
  // Store config only. The GroundSurface is (re)configured + rebuilt in
  // setStaticCloud, so a live reconfigure of band/decay/inflation knobs never
  // wipes the accumulated ground surface.
  void setConfig(const CostmapConfig& cfg) { cfg_ = cfg; }

  // Foot-contact ground scalar (map frame), supplied by the node from TF. Used
  // as the GroundSurface fallback for cells with no trusted per-cell ground.
  void setFootGround(double ground_z) {
    foot_ground_ = ground_z;
    has_foot_ = true;
  }
  bool hasFootGround() const { return has_foot_; }

  // Rebuild the ground surface + static grid geometry from a map-frame cloud
  // (bare projection, no raycast; used by tests). Returns false if empty.
  bool setStaticCloud(const std::vector<Eigen::Vector3f>& points_map);

  // Production path: rebuild the ground surface + static grid from the backend's
  // per-keyframe clouds, raycasting free space from each keyframe origin. Returns
  // false (state unchanged) if there are no points.
  bool setStaticKeyframes(const std::vector<StaticKeyframe>& kfs);

  bool hasStatic() const { return static_grid_.initialized(); }

  // Clear the dynamic (transient-obstacle) layer: resets it to all NO_INFORMATION
  // so the next fuse() sees static-only. Does NOT touch the static grid, ground
  // surface, or static probability. A still-present obstacle reappears on the
  // very next dynamic update -- this clears stale/transient marks, not walls.
  void clearDynamic();

  // Integrate a live map-frame scan into the dynamic layer (no-op until a static
  // cloud has fixed the geometry).
  void updateDynamic(const std::vector<Eigen::Vector3f>& points_map,
                     const Eigen::Vector3f& sensor_origin, double now);

  // Combine static + dynamic (obstacle precedence, then free), inflate, return.
  const CostmapGrid& fuse();
  const CostmapGrid& master() const { return master_; }
  // The fused 3-state occupancy BEFORE inflation (free / obstacle / unknown),
  // the 3-state grid map. Valid after fuse(); the node publishes it for the
  // standard rviz/Map "map" colour scheme. master() carries the inflated costmap.
  const CostmapGrid& occupancy() const { return occupancy_; }
  const GroundSurface& groundSurface() const { return surface_; }
  // Per-cell occupancy probability of the static log-odds map, aligned to the
  // master grid (nav_msgs/OccupancyGrid value: -1 unknown, else 0..100). Empty
  // until setStaticKeyframes has run (setStaticCloud is a bare projection).
  const std::vector<std::int8_t>& staticProbability() const { return static_prob_; }
  // Per-cell occupancy probability of the dynamic CHANGE evidence, aligned to the
  // master grid (nav_msgs/OccupancyGrid value: -1 neutral/untouched, else 0..100).
  // Valid after fuse(); a diagnostic for tuning the add/clear thresholds.
  const std::vector<std::int8_t>& changeProbability() const { return change_prob_; }

 private:
  int obstacleMaxLayers() const {
    return static_cast<int>(std::lround(cfg_.obstacle_max_height / cfg_.resolution));
  }
  // Fill a StaticLayerParams from the config (degrees -> radians for the gate).
  StaticLayerParams staticParams() const;
  void buildChangeProb();  // fill change_prob_ from the dynamic layer log-odds
  GroundLookup groundLookup() const {
    return GroundLookup{&surface_, foot_ground_, has_foot_};
  }

  CostmapConfig cfg_;
  GroundSurface surface_;
  double foot_ground_ = 0.0;
  bool has_foot_ = false;
  StaticLayer static_layer_;
  InflationLayer inflation_layer_;
  DynamicLayer dynamic_layer_;
  CostmapGrid static_grid_;  // 3-state projection of the SLAM map
  CostmapGrid occupancy_;    // fused 3-state, pre-inflation (3-state grid map)
  CostmapGrid master_;       // fused + inflated, published
  std::vector<std::int8_t> static_prob_;  // static occupancy probability (-1/0..100)
  std::vector<std::int8_t> change_prob_;  // change-evidence probability (-1/0..100)
};

}  // namespace g1_costmap
