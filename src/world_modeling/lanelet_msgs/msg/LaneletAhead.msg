# LaneletAhead.msg
# Legally reachable lanelets ahead of ego within a configurable radius.
#
# Uses BFS through the routing graph (successors + legal lane changes) from
# the current lanelet. Only includes forward-reachable, same-direction,
# legally-connected lanelets within the configured radius.

std_msgs/Header header

# Full lanelet data (IDs in each lanelet)
Lanelet[] lanelets

# Ego's current lanelet (-1 if unknown)
int64 current_lanelet_id

# Configured radius used (meters)
float64 radius_m
