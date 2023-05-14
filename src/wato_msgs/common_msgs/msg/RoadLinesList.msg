std_msgs/Header header

geometry_msgs/Pose pose

ParkingSpot[] parking_spots
LaneLine[] lane_lines
LaneLine[] trajectory_lines

# Stop lines are ordered from closest to furthest (along the car's current lane)
StopLine[] stop_lines

StopLine[] crosswalks
