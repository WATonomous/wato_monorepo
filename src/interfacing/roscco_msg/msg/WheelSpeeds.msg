std_msgs/Header header
float32 ne
float32 nw
float32 se
float32 sw

# 4 separate floats and not an array 
# because you need to model each front wheel
# differently through ackermann frames in junction
# with your steering angle to determine odom

# Also for clarity
# obviously i.e. NE / north-east is the front-right wheel
