# just placeholder sample tracking msg for now
std_msgs/Header header

# Identifier for a tracked object (if applicable)
int32 id

# 2D bounding box center (pixels) and size (pixels)
float32 cx
float32 cy
float32 width
float32 height

# Optional confidence score [0.0, 1.0]
float32 score

# Optional class label
string class_name

