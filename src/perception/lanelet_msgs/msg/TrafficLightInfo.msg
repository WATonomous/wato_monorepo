# TrafficLightInfo.msg
# Information about a traffic light and its associated stop line.
#
# Used to identify traffic lights that control a lanelet and where
# the vehicle should stop when the light is red.

# Unique identifier for this traffic light
int64 id

# 3D position of the traffic light in map frame
geometry_msgs/Point position

# ID of the associated stop line (-1 if none)
# The stop line is where the vehicle should stop when light is red
int64 stop_line_id
