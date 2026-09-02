uint8 SOURCE_OBJECT=0
uint8 SOURCE_USER=1
string name
string map_name
geometry_msgs/Pose pose                 # resolved world pose (map frame)
uint32 anchor_keyframe_id
geometry_msgs/Pose anchor_relative_pose # T_keyframe_navpoint
uint8 source
