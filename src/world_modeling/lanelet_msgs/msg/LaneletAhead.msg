# LaneletAhead.msg
# All lanelets within a configurable radius from ego, regardless of route.
#
# Unlike RouteAhead which returns lanelets ON THE ACTIVE ROUTE,
# this returns ALL lanelets within the configured radius for general awareness.

std_msgs/Header header

# Full lanelet data (IDs in each lanelet)
Lanelet[] lanelets

# Ego's current lanelet (-1 if unknown)
int64 current_lanelet_id

# Configured radius used (meters)
float64 radius_m
