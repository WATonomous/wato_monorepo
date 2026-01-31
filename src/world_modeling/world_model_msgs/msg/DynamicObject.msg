# DynamicObject.msg
# A single dynamic object with full state information.
#
# Contains ID, type, pose, size, lanelet context, and predicted paths.

std_msgs/Header header
int64 id

# Entity Type
uint8 TYPE_UNKNOWN=0
uint8 TYPE_CAR=1
uint8 TYPE_HUMAN=2
uint8 TYPE_BICYCLE=3
uint8 TYPE_MOTORCYCLE=4
uint8 TYPE_TRAFFIC_LIGHT=5

uint8 entity_type

# State
geometry_msgs/Pose pose
geometry_msgs/Vector3 size

# Lanelet the object is currently on (-1 if unknown)
int64 lanelet_id

# When this object was last detected
builtin_interfaces/Time detection_timestamp

# Historical Path (most recent first, oldest last)
geometry_msgs/PoseStamped[] history

# Predictions (from prediction node)
Prediction[] predictions
