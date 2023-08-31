std_msgs/Header header

# Vehicle State
#######################

float64 forward_speed # m/s
float64 forward_accel # m/s^2
float64 wheel_angle   # degrees (not sure if we want/need this)
geometry_msgs/Pose global_pose # in meters from where odometer was started up


common_msgs/Obstacle[] obstacles
common_msgs/StopLine[] stop_lines
common_msgs/TrafficLight[] traffic_lights
common_msgs/TrafficSign[] traffic_signs
common_msgs/ParkingSpot[] parking_spots
common_msgs/RoadLinesList lines