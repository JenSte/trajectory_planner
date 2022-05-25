# The pose itself.
geometry_msgs/PoseStamped pose

# The index of the linear velocity (only used for 5D paths).
int32 linear_index

# The index of the angular velocity (only used for 5D paths).
int32 angular_index

# The cost of the pose on the path.
float64 cost

# The heuristic value of the pose
float64 heuristic
