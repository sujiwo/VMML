Header header

# Camera parameter
uint32 width
uint32 height
float64[9] K

# Keyframe
# Pose of each keyframe
geometry_msgs/Pose[] keyframe_pose

# Map point
# Positions of each map point
geometry_msgs/Point map_point

# Keyframe database
