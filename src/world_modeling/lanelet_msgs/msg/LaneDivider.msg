# LaneDivider.msg
# A lane divider line between two adjacent lanes.

# Sampled points along the divider
geometry_msgs/Point[] points

# Boundary type for each segment (length = points.length - 1)
# Values: "solid_white", "dashed_white", "solid_yellow", "dashed_yellow", "virtual"
string[] boundary_types
