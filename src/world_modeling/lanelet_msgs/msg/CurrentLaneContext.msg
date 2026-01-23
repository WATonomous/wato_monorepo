# CurrentLaneContext.msg
# Real-time lane context for the ego vehicle, published as a topic.
#
# Published by the lanelet node at ~20Hz using TF (map -> base_link).
# Downstream nodes (behavior tree, planner) subscribe for low-latency
# lane context without service call overhead.

std_msgs/Header header

# Ego Position

# Distance along lanelet centerline from its start (meters)
float64 arc_length

# Signed lateral offset from centerline (meters)
# Positive = left of centerline, Negative = right
float64 lateral_offset

# Heading error relative to lane direction (radians)
# Positive = pointing left of lane, Negative = pointing right
float64 heading_error

# Distance to end of current lanelet (meters)
float64 distance_to_lanelet_end_m

# Current Lanelet
Lanelet current_lanelet

# Distances to Upcoming Events
# Distance from current position. Value is -1.0 if not present within lookahead.
float64 distance_to_intersection_m
float64 distance_to_traffic_light_m
float64 distance_to_stop_line_m
float64 distance_to_yield_m

# Upcoming Path
# Next lanelets along current route (empty if no route active)
int64[] upcoming_lanelet_ids
float64[] upcoming_lanelet_distances_m
