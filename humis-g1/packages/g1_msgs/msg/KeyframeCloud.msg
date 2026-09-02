# One SLAM keyframe for costmap raycasting: its sensor origin and its own cloud,
# both in the map frame (at the optimized pose). The costmap raycasts from origin
# through each return to clear the swept free space.
uint64 id
geometry_msgs/Point origin
sensor_msgs/PointCloud2 cloud
