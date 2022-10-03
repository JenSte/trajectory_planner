# The linear velocities used for the 5D planning, in m/s.
float64[] linear_velocities

# The angular velocities used for the 5D planning, in rad/s.
float64[] angular_velocities

# Two arrays of the same length, where the two values at the
# same position are a valid entry in the motion model. (As
# not each linear/angular velocity combination must be valid.)
int32[] linear_indices
int32[] angular_indices
