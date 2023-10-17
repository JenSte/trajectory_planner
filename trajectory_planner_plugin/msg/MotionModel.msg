# The maximum wheel velocity used to create this motion
# model, in m/s.
float64 maximum_wheel_velocity

# The maximum wheel acceleration used to create this motion
# model, in m/(s^2).
float64 maximum_wheel_acceleration

# The distance between the wheels of the robot used to create
# this motion model, in m.
float64 wheel_distance

# The linear velocities used for the 5D planning, in m/s.
float64[] linear_velocities

# The angular velocities used for the 5D planning, in rad/s.
float64[] angular_velocities

# Two arrays of the same length, where the two values at the
# same position are a valid entry in the motion model. (As
# not each linear/angular velocity combination must be valid.)
int32[] linear_indices
int32[] angular_indices
