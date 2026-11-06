# Rich status/telemetry from a trajectory-tracking controller (e.g. pure pursuit).
# Published every control cycle so tracking quality and disengagement can be
# monitored live and inspected in bags.

std_msgs/Header header

# --- Tracking error (in the controller reference frame) ---
float64 cross_track_error    # signed lateral distance to nearest path segment (m), left positive
float64 heading_error        # signed angle between vehicle heading and path tangent (rad)

# --- Controller internals for this cycle ---
float64 lookahead_distance   # effective lookahead distance used (m)
float64 path_curvature       # estimated path curvature ahead (1/m)
float64 commanded_speed      # speed command sent (m/s)
float64 commanded_steering   # steering command sent (rad)

# --- Safety / engagement ---
bool disengaged              # true when tracking thresholds were tripped
string reason                # why: "", "ok", "cte_exceeded", "heading_exceeded", "no_lookahead_point"
