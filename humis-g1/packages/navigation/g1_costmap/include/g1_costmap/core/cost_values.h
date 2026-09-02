#pragma once

#include <cstdint>

namespace g1_costmap {

// Internal cost convention (costmap_2d style). The ROS node maps these to the
// nav_msgs/OccupancyGrid [0,100] / -1 range on publish (see costToOccupancy).
static constexpr std::uint8_t FREE_SPACE = 0;
static constexpr std::uint8_t INSCRIBED_INFLATED = 253;  // within the robot radius
static constexpr std::uint8_t LETHAL_OBSTACLE = 254;     // an actual obstacle cell
static constexpr std::uint8_t NO_INFORMATION = 255;      // unknown

// Map an internal cost to the nav_msgs/OccupancyGrid range. The three blocking
// levels are DISTINCT so downstream can tell a real obstacle from the inscribed
// inflation band: 254 (LETHAL) -> 100, 253 (INSCRIBED) -> 99, the inflation band
// [1,252] -> [1,98] (monotonic non-decreasing), 0 -> 0, 255 (unknown) -> -1.
inline std::int8_t costToOccupancy(std::uint8_t cost) {
  if (cost == FREE_SPACE) return 0;
  if (cost == NO_INFORMATION) return -1;
  if (cost == LETHAL_OBSTACLE) return 100;     // real obstacle
  if (cost == INSCRIBED_INFLATED) return 99;   // within robot_radius (center-blocked)
  // cost in [1, INSCRIBED_INFLATED-1] -> [1, 98], monotonic non-decreasing.
  return static_cast<std::int8_t>(1 + (97 * (cost - 1)) / (INSCRIBED_INFLATED - 1));
}

}  // namespace g1_costmap
