# Obstacle Type Enums
string OBS_TP_UNKNOWN=UNKNOWN
string OBS_TP_PED=PEDESTRIAN
string OBS_TP_CYC=CYCLIST
string OBS_TP_CAR=CAR
string OBS_TP_TRUCK=TRUCK
string OBS_TP_TRAFLIGHT=TRAFLIGHT
string OBS_TP_BUS=BUS
string OBS_TP_MBIKE=MBIKE
string label # see Obstacle Type Enums
# Detection confidence
float32 confidence
# Position and its uncertainty
# When the message is sent to Processing it's in vehicle POV coordinate system, but then Processing transforms it to novatel coordinate system
# Coordinate system documentation: https://phabricator.watonomous.ca/w/integration/data_point_physical_representation/
# For 3d bounding boxes, the (x, y, z) is the center point of the 3d bounding box
# For 2d bounding boxes, the (x, y) is the top left point of the 2d bounding box
# geometry_msgs/PoseWithCovariance pose
# Velocity and its uncertainty
# geometry_msgs/TwistWithCovariance twist 
# Dimensions of bounding box assuming BEV perspective
# x=width, y=height, z=depth
# For example, a vehicle with its bumper facing North that's
# encapsulated by a rectangular prism defines width as the
# E/W measurement, height as the N/S measurement.
# Obstacle.msg is also used as 2d bounding boxes
# In that case width_along_x_axis is the width of the 2d bounding box in image coordinates
# and height_along_y_axis is the height of the 2d bounding box in image coordinates
# z axis is not used with 2d bounding boxes
float64 width_along_x_axis
float64 height_along_y_axis
float64 depth_along_z_axis
# Unique ID number
uint32 object_id