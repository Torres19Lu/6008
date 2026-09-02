#pragma once

#include <algorithm>
#include <cmath>

namespace g1_costmap {

// Floor-snap a metric height to an integer layer index at the given cell pitch.
// std::floor handles the negative half-line: layerOf(-0.001, 0.1) == -1.
inline int layerOf(double z, double cell_size) {
  return static_cast<int>(std::floor(z / cell_size));
}

// Outcome of classifying a point's height against its cell's ground, in layers.
// IGNORED means above the obstacle band (ceiling / high ghost): the cell is not
// marked, so it stays unknown unless another point reaches FREE/OBSTACLE.
enum class Tier3 { FREE, OBSTACLE, IGNORED };

// Ground-relative band (owner model "below ground = free, ground up to lidar =
// obstacle, above = ignore"):
//   signed_offset <= walkable_layers                   -> FREE (at/below ground)
//   walkable_layers < signed_offset <= obstacle_max    -> OBSTACLE
//   signed_offset > obstacle_max                        -> IGNORED
// signed_offset = layerOf(point_z) - layerOf(ground_z), both at the cell pitch.
inline Tier3 classify3(int signed_offset, int walkable_layers,
                       int obstacle_max_layers) {
  if (signed_offset <= walkable_layers) return Tier3::FREE;
  if (signed_offset <= obstacle_max_layers) return Tier3::OBSTACLE;
  return Tier3::IGNORED;
}

// Normal-gated refinement of classify3. `normal_angle_rad` is the angle between
// the point's surface normal and the vertical (0 = horizontal surface / flat
// ground, pi/2 = vertical surface / wall face). With `has_normal` false this is
// exactly classify3 (pure height bands). The gate (owner choice: "near-horizontal
// -> ground, near-vertical -> obstacle") refines the height result in two cases,
// making the band robust to SLAM Z drift and slope:
//   - a FLAT surface (angle <= ground_angle_rad) only a few layers above the
//     walkable band is demoted OBSTACLE -> FREE: it is drifted/uneven floor, not
//     an object (a real flat-topped obstacle sits far above the band and stays).
//   - a VERTICAL surface (angle >= vertical_angle_rad) at or above the ground
//     layer is promoted FREE -> OBSTACLE: a wall/obstacle face on the floor.
// Below-ground points and IGNORED (ceiling) returns are never reclassified.
inline Tier3 classify3WithNormal(int signed_offset, int walkable_layers,
                                 int obstacle_max_layers, bool has_normal,
                                 double normal_angle_rad, double ground_angle_rad,
                                 double vertical_angle_rad, int flat_grace_layers) {
  const Tier3 base =
      classify3(signed_offset, walkable_layers, obstacle_max_layers);
  if (!has_normal) return base;
  const bool is_flat = normal_angle_rad <= ground_angle_rad;
  const bool is_vertical = normal_angle_rad >= vertical_angle_rad;
  if (base == Tier3::OBSTACLE && is_flat &&
      signed_offset <= walkable_layers + flat_grace_layers) {
    return Tier3::FREE;  // flat low surface = floor drift, not an obstacle
  }
  if (base == Tier3::FREE && is_vertical && signed_offset >= 0) {
    return Tier3::OBSTACLE;  // vertical face standing on the ground
  }
  return base;
}

// Angle (rad) between a surface normal and the vertical, from the normal's z
// component magnitude: |nz|=1 -> 0 (horizontal surface), |nz|=0 -> pi/2 (vertical).
// Returns has_normal=false for a zero/degenerate normal (gate then falls back to
// pure height bands).
inline double normalAngleFromUp(double nx, double ny, double nz, bool& has_normal) {
  const double n = std::sqrt(nx * nx + ny * ny + nz * nz);
  if (!(n > 1e-6)) {
    has_normal = false;
    return 0.0;
  }
  has_normal = true;
  double c = std::abs(nz) / n;
  if (c > 1.0) c = 1.0;
  return std::acos(c);
}

// Obstacle band TOP, in layers above the cell ground. The domain is "ground ->
// lidar plane": a return above `lidar_z - phantom_drop`
// (the sensor plane, minus a small drop for inverted-Mid-360 upward ghosts) is a
// ceiling/overhead the robot walks under, not an obstacle. A fixed
// `obstacle_max_layers` caps it as a safety bound when the (drift-prone) lidar z
// runs high; if the lidar plane degenerates to at/below ground (bad pose z) it is
// ignored and the fixed cap is used.
inline int bandTopLayers(double lidar_z, double ground_z, double phantom_drop,
                         int obstacle_max_layers, double cell_size) {
  const int top_lidar =
      layerOf(lidar_z - phantom_drop, cell_size) - layerOf(ground_z, cell_size);
  if (top_lidar < 1) return obstacle_max_layers;  // degenerate lidar z -> safety cap
  return std::min(top_lidar, obstacle_max_layers);
}

}  // namespace g1_costmap
