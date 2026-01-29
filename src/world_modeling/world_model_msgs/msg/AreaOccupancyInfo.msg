# AreaOccupancyInfo.msg
# Occupancy status for a single defined area.

std_msgs/Header header           # Timestamp and frame_id for this area

string name
bool is_occupied
DynamicObject[] objects
