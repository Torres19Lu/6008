#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "g1_costmap/core/costmap_grid.h"
#include "g1_costmap/core/ground_surface.h"
#include "g1_costmap/core/occupancy_accumulator.h"

namespace g1_costmap {

struct StaticLayerParams {
  double resolution = 0.1;        // m / cell
  double margin = 1.0;            // m of padding around the cloud XY bounds
  int walkable_layers = 1;        // |offset| <= this around ground -> FREE
  int obstacle_max_layers = 20;   // safety cap above ground; the lidar plane normally bites first
  double phantom_drop = 0.1;      // m below the keyframe lidar plane; above it -> ignored (ceiling)
  int hole_fill_iters = 2;        // morphological-close passes over UNKNOWN holes (0 off)
  double xy_clip_pct = 0.005;     // drop this XY fraction each side for robust grid bounds
  double max_range = 0.0;         // m; raycast beam range cap, 0 = unlimited

  // Normal gate (refines the height band): a near-horizontal surface just above
  // ground reads FREE (floor drift), a near-vertical surface on the floor reads
  // OBSTACLE. Disabled when a keyframe carries no normals.
  bool use_normals = true;
  double ground_normal_angle = 0.698;   // rad (~40 deg); <= this is flat
  double vertical_normal_angle = 1.047; // rad (~60 deg); >= this is vertical
  int normal_flat_grace_layers = 2;     // flat surfaces up to walkable+this demote

  // Probabilistic (log-odds) accumulation across keyframes.
  OccupancyAccumulatorParams occ;
  bool erode_obstacles = true;          // remove isolated obstacle speckle

  // Angular gap fill: per keyframe, fill the unobserved wedges in front of the
  // observed surfaces with FREE evidence (kills the "punched-dot" unknown holes).
  bool angular_fill = true;
  double angular_fill_step = 0.0087;    // rad (~0.5 deg) angular bin width
};

// One keyframe for raycasting: its sensor origin, its own returns, and (optional)
// per-return surface normals, all in the map frame (at the optimized pose). When
// `normals` is empty the normal gate is skipped (pure height bands).
struct StaticKeyframe {
  Eigen::Vector3f origin;
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector3f> normals;  // parallel to points, or empty
};

// Remove isolated obstacle speckle in-place (obstacle erosion): an OBSTACLE
// cell with >= 3 FREE 4-neighbours and no UNKNOWN 4-neighbour becomes FREE. This
// clears single noise obstacles surrounded by open free space while preserving
// obstacles that border unknown (walls, map frontier). One pass.
void erodeObstacleSpeckle(CostmapGrid& g);

// Projects the static SLAM map (map-frame 3D points) to a fresh 3-state grid:
// the grid is sized from the cloud XY bounds + margin and filled UNKNOWN; each
// point is classified by its height offset from the per-cell ground (via the
// GroundLookup) into FREE (at/below ground) or OBSTACLE (ground up to the band
// top); above the band is ignored. OBSTACLE takes precedence within a cell.
class StaticLayer {
 public:
  // Bare projection (last-write, no raycast); used by tests. Fills `out`
  // (UNKNOWN / FREE / OBSTACLE) and returns true; false (out left uninitialized)
  // when there are no points to bound the grid.
  bool build(const std::vector<Eigen::Vector3f>& points_map,
             const GroundLookup& ground, const StaticLayerParams& p,
             CostmapGrid& out) const;

  // Production path: per keyframe build a local grid (segment ground/obstacle
  // with the normal gate, raycast free space from the origin, angular-fill the
  // unobserved wedges), accumulate each local grid into a log-odds global map,
  // then threshold to 3-state, erode obstacle speckle, and morphologically close
  // residual holes. When `prob_out` is non-null it is filled with the per-cell
  // occupancy probability (nav_msgs/OccupancyGrid value: -1 unknown, else 0..100)
  // BEFORE the erosion/close cosmetics, so it reflects the raw fusion confidence.
  bool buildFromKeyframes(const std::vector<StaticKeyframe>& kfs,
                          const GroundLookup& ground, const StaticLayerParams& p,
                          CostmapGrid& out,
                          std::vector<std::int8_t>* prob_out = nullptr) const;
};

}  // namespace g1_costmap
