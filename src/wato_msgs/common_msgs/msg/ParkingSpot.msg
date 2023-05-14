# A parking spot is described by the lanelet formed by the parking lines.
# The lanelet is described by 2 points, "entry" and "end".
# The "entry" and "end" point form a line parallel to the parking spot heading.

std_msgs/Header header

# The following two points represent a line (parallel to parking lines) cutting through the middle of the parking spot.

# Point of the mouth of the parking spot.
geometry_msgs/Point entry

# Expected point of vehicle's front bumper when parked.
geometry_msgs/Point end
