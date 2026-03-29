# Lanelets ahead of ego that are part of the active route.
# Enables BFS via Lanelet.successor_ids for geometric map building.

std_msgs/Header header

# Lanelet IDs in route traversal order
int64[] ids

# Full lanelet data (geometry, connectivity via successor_ids)
Lanelet[] lanelets

# Arc-length from ego to first lanelet start
float64 distance_to_first_m

# Total distance covered by these lanelets
float64 total_distance_m

# True if active route exists
bool has_active_route
