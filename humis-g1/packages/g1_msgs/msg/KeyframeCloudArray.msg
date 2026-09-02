# The full set of SLAM keyframes (map frame). Republished latched on optimization
# and on map load so a costmap can rebuild its static layer from the up-to-date
# optimized poses (loop closure re-optimizes every keyframe).
std_msgs/Header header
g1_msgs/KeyframeCloud[] keyframes
