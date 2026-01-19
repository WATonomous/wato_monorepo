# LanePath.msg
# A single lane within a corridor, with its centerline and width.

# Lanelet IDs that make up this lane (in traversal order)
int64[] lanelet_ids

# Sampled centerline points
geometry_msgs/Point[] centerline

# Lane width at each centerline point (meters)
float64[] widths
