# Corridor.msg
# Planning corridor for lattice-based motion planning.
#
# Contains the reference lane (route) and immediate adjacent lanes
# for lane change maneuvers. All lanes are sampled at the same
# arc-length intervals along the reference path.
#
# Structure:
#
#     [left_lane.left boundary]
#     ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─   left_lane centerline
#     ═══════════════════════════   reference.left boundary (dashed if crossable)
#          reference centerline   ← the route
#     ═══════════════════════════   reference.right boundary (dashed if crossable)
#     ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─   right_lane centerline
#     [right_lane.right boundary]

# ===========================================================================
# Reference Lane (the route)
# ===========================================================================

CorridorLane reference

# ===========================================================================
# Adjacent Lanes (for lane changes)
# ===========================================================================
# Arrays are empty if no crossable adjacent lane exists.
# May have fewer samples than reference if adjacent lane ends early.

CorridorLane left_lane
CorridorLane right_lane
