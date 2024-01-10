# Some traffic lights may display more than one signal.
# The signal to react to depends on the position of the vehicle
# (e.g. which lane it's in).

std_msgs/Header header

# Traffic Light State Enums
string TL_ST_NON=NON
string TL_ST_RED=RED
string TL_ST_YEL=YELLOW
string TL_ST_GRE=GREEN
string TL_ST_FLA=FLASHING_RED
string left
string forward
string right

# Traffic Light Sign Direction Enums
string TL_DIR_NON=NON
string TL_DIR_LT=LEFT
string TL_DIR_RT=RIGHT
string TL_DIR_FD=FORWARD
string TL_DIR_LT_FD=LEFT_FORWARD
string TL_DIR_RT_FD=RIGHT_FORWARD
string sign_dir

# Will be used as follows:
# (pose.position.x, pose.position.y) is the top left point of the bounding box (in image coordinates)
# (dimensions.x, dimensions.y) will be the width and height of the bounding box (in image coordinates) 
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions

# Will be used as follows:
# (p1.x, p1.y, p1.z) is a corner point of the 3d bounding box describing the shape of the traffic light, opposite to p2 
# (p2.x, p2.y, p2.z) is a corner point of the 3d bounding box describing the shape of the traffic light, opposite to p1
# both points are provided in the odom frame
geometry_msgs/Point p1
geometry_msgs/Point p2

StopLine stop_line

# Unique ID number
uint64 id
