std_msgs/Header header

# Traffic Sign Enums
string TS_TP_UNKNOWN=UNKNOWN
string TS_TP_LEFT=LEFT_TURN_SIGN
string TS_TP_RIGHT=RIGHT_TURN_SIGN
string TS_TP_NO_LEFT=NO_LEFT_TURN_SIGN
string TS_TP_NO_RIGHT=NO_RIGHT_TURN_SIGN
string TS_TP_STOP=STOP_SIGN
string TS_TP_5MPH=5MPH_SIGN
string TS_TP_10MPH=10MPH_SIGN
string TS_TP_15MPH=15MPH_SIGN
string TS_TP_20MPH=20MPH_SIGN
string TS_TP_25MPH=25MPH_SIGN
string TS_TP_HANDICAP=HANDICAP_SIGN
string TS_TP_PED=PED_CROSSING
string TS_TP_RR=RAILROAD_CROSSING
string TS_TP_DNE=DO_NOT_ENTER
string TS_TP_PARK=PARKING_SIGN
string traffic_sign_type

# Detection confidence
float32 confidence

# Will be used as follows:
# (pose.position.x, pose.position.y) is the top left point of the bounding box (in image coordinates)
# (dimensions.x, dimensions.y) will be the width and height of the bounding box (in image coordinates) 
# After HLDF, this msgs will contain 3d box data
# Will be used as follows:
# (pose.position.x, pose.position.y, pose.position.z) is the center point of the 3d bounding box (in novatel coordinates)
# (dimensions.x, dimensions.y, dimensions.z) will be the depth, width and height of the 3d bounding box (in novatel coordinates) 
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions

# Valid only when traffic_sign_type=STOP
StopLine stop_line

# Unique ID number
uint32 id
