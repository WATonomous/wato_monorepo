# PathArray.msg
# Contains all the generated paths along with their costs

std_msgs/Header header

# List of all available paths
nav_msgs/Path[] paths

# Corresponding list of cost for each path
float64[] costs