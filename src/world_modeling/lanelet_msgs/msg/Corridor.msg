# Corridor.msg
# Multi-lane driving corridor for motion planning.
#
# Provides sampled geometry for multiple parallel lanes, allowing the
# planner to generate trajectories that cross lane boundaries. All arrays
# are sampled at the same arc length intervals.
#
# Structure (3-lane example):
#
#     leftmost_boundary
#     ═══════════════════════════════════════
#          lanes[0].centerline
#     ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─   dividers[0] (dashed)
#          lanes[1].centerline  ← reference_lane_index = 1
#     ─────────────────────────────────────   dividers[1] (solid)
#          lanes[2].centerline
#     ═══════════════════════════════════════
#     rightmost_boundary

# ===========================================================================
# Lane Structure
# ===========================================================================

# Number of lanes in this corridor
int32 num_lanes

# Index of the reference lane (the route lane)
# Lanes with index < reference are to the left
# Lanes with index > reference are to the right
int32 reference_lane_index

# ===========================================================================
# Outer Boundaries
# ===========================================================================
# Hard constraints - vehicle must stay within these

geometry_msgs/Point[] leftmost_boundary
geometry_msgs/Point[] rightmost_boundary

# ===========================================================================
# Per-Lane Data
# ===========================================================================
# lanes[i] contains centerline and widths for lane i
# Ordered left-to-right (lanes[0] is leftmost)

LanePath[] lanes

# ===========================================================================
# Lane Dividers
# ===========================================================================
# dividers[i] is the boundary between lanes[i] and lanes[i+1]
# Length is (num_lanes - 1)
# Use boundary_types to determine if crossing is allowed

LaneDivider[] dividers

# ===========================================================================
# Reference Path Metadata
# ===========================================================================
# Indexed along lanes[reference_lane_index].centerline

# Arc length from corridor start (meters)
float64[] arc_length

# Tangent heading at each point (radians, 0 = +X, CCW positive)
float64[] heading

# Signed curvature at each point (1/meters, positive = left)
float64[] curvature
