std_msgs/Header header
float32 NE
float32 NW
float32 SE
float32 SW

# 4 separate floats and not an array 
# because you need to model each front wheel
# differently through ackermann frames in junction
# with your steering angle to determine odom

# Also for clarity
# obviously i.e. NE / north-east is the front-right wheel
