# Lanelet.msg
# Complete data for a single lanelet (atomic road segment).
#
# Contains geometry, semantics, connectivity, and regulatory elements.
# Used in GetShortestRoute responses, CurrentLaneContext, and MapVisualization.

int64 id

# Geometry (raw boundary points, not sampled, direct from map)
geometry_msgs/Point[] left_boundary
geometry_msgs/Point[] right_boundary
geometry_msgs/Point[] centerline

# Curvature at each centerline point (1/m, same length as centerline).
# Positive = curving left, negative = curving right.
# Computed via Menger curvature on three consecutive centerline points.
float64[] centerline_curvature

# Boundary Attributes
uint8 BOUNDARY_DASHED=0
uint8 BOUNDARY_SOLID=1
uint8 BOUNDARY_SOLID_SOLID=2
uint8 BOUNDARY_ROAD_EDGE=3

# Boundary color constants
uint8 COLOR_WHITE=0
uint8 COLOR_YELLOW=1

# Boundary metadata (type and color for left/right boundaries)
uint8 left_boundary_type
uint8 right_boundary_type
uint8 left_boundary_color
uint8 right_boundary_color

# Type: "road", "intersection", "crosswalk", "parking", "bicycle_lane"
string lanelet_type

# Semantics
# Speed limit in meters per second
float64 speed_limit_mps

# True if mandatory (legal limit), false if advisory
bool speed_limit_is_mandatory

# Intersection flags
bool is_intersection
bool has_traffic_light
bool has_stop_line
bool has_yield_sign
bool must_yield

# Lanelets this one must yield to
int64[] yield_to_lanelet_ids

# Lanelets that cross/conflict with this one
int64[] conflicting_lanelet_ids

# Connectivity
# Lane change permissions
bool can_change_left
bool can_change_right

# Adjacent lanelet IDs (-1 if none)
int64 left_lane_id
int64 right_lane_id

# Successor lanelet IDs (what comes after this lanelet)
int64[] successor_ids

# Regulatory Elements
TrafficLightInfo[] traffic_lights
StopLineInfo[] stop_lines
