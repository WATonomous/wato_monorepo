# CorridorLane.msg
# A single lane within a planning corridor, with sampled geometry and boundaries.
#
# All arrays are sampled at the same arc-length intervals.
# Arrays may be empty if this lane doesn't exist (e.g., no adjacent lane).

# ===========================================================================
# Path Geometry
# ===========================================================================

# Sampled centerline points in map frame
geometry_msgs/Point[] centerline

# Arc length from corridor start at each sample (meters)
float64[] arc_length

# Tangent heading at each sample (radians, 0 = +X, CCW positive)
float64[] heading

# Signed curvature at each sample (1/meters, positive = curving left)
float64[] curvature

# Speed limit at each sample (m/s)
float64[] speed_limit

# ===========================================================================
# Lane Boundaries
# ===========================================================================
# Boundaries are expressed as lateral offsets from centerline.
# Positive = left of centerline, Negative = right of centerline.

# Boundary type constants
uint8 BOUNDARY_DASHED=0     # Can cross (lane change allowed)
uint8 BOUNDARY_SOLID=1      # Cannot cross (lane change prohibited)
uint8 BOUNDARY_ROAD_EDGE=2  # Hard boundary (edge of road)

# Left boundary
float64[] left_offset
uint8[] left_boundary_type

# Right boundary
float64[] right_offset
uint8[] right_boundary_type
