# StopLineInfo.msg
# Information about a stop line on the road.
#
# Stop lines indicate where a vehicle must stop for traffic lights,
# stop signs, or yield situations.

# Unique identifier for this stop line
int64 id

# The stop line geometry as a sequence of points (typically 2 points
# defining a line across the lane, but may have more for curved roads)
geometry_msgs/Point[] points

# Distance from the start of the lanelet to this stop line (meters)
# Useful for calculating when to begin braking
float64 distance_along_lanelet_m
