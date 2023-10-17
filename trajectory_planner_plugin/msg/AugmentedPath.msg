# The name of the path, used to distinguish between different messages
# published for different parts of the path search.
string name

# Possible values for 'segment_type'.
int32 SEGMENT_TYPE_FORWARD=0
int32 SEGMENT_TYPE_BACKWARD=1
int32 SEGMENT_TYPE_TURN=2

# The type of the segment, either 'SEGMENT_TYPE_FORWARD' or 'SEGMENT_TYPE_BACKWARD'
# denoting that the poses on the path shall be followed in forward or backward
# direction, respectively. A "turn segment" is only used to turn the robot on a fixed
# x/y coordinate.
#
# Only used for segments created by the 5D planner, for 3D planner segments this is
# always set to 'SEGMENT_TYPE_FORWARD'.
int32 segment_type

# The number of the segment in all the segments that are created.
int32 segment_count

# The total number of segments that were created for planning from start to goal.
int32 total_segments

# The augmented poses that make up the path.
#
# In case this segment is a "turn" segment, this array consists of two poses, that
# have the same x/y coordinates and differ only in the orientation.
AugmentedPose[] poses

# The motion model used for the planning
MotionModel motion_model
