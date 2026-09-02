# Lightweight keyframe poses (no clouds), latched on /slam/keyframe_poses,
# republished on intake/optimization/load. Manager anchors nav points to these.
std_msgs/Header header
g1_msgs/KeyframePose[] poses
