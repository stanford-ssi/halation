# Message for sorted segment of points.

# Each segment is assigned an id, which uniquely identifies the segment consistently across multiple frames.
# The segments can be linked to previous and next segment using lastPointPriorSeg and firstPointNextSeg, respectively.
# Segment messages can be used as an input to a classifier such as a person detector, or for annotating
# sensor_msgs/Laserscan messages with groundtruth person tracks.

# ID of the segment
uint64 id 0

# Label of the segment
uint32 label 0

# Angular distance to the closest boundary (in rads)
float64 angular_distance 0.0

# Last point of the previous segment
geometry_msgs/Point last_point_prior_segment

# First point of the next segment
geometry_msgs/Point first_point_next_segment

# Points of the segment
geometry_msgs/Point[] points

# Type of the labels
uint32 BACKGROUND = 0
uint32 PERSON = 1
uint32 PERSON_CANE = 2
uint32 PERSON_WHEEL_CHAIR = 3
