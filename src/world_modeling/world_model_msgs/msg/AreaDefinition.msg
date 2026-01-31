# AreaDefinition.msg
# Defines a geometric area for occupancy detection.
#
# Areas are defined relative to a reference frame (typically base_link).

string name

# Area type constants
uint8 TYPE_CIRCLE=0
uint8 TYPE_PARTIAL_CIRCLE=1
uint8 TYPE_RECTANGLE=2

uint8 area_type

# All areas: center relative to reference frame
float64 center_x
float64 center_y

# Circle/PartialCircle: radius in meters
float64 radius

# PartialCircle only (radians, 0 = forward/+x, CCW positive)
float64 start_angle
float64 end_angle

# Rectangle: dimensions in meters
float64 length  # x-dimension
float64 width   # y-dimension
