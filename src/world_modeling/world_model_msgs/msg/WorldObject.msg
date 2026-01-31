# WorldObject.msg
# A tracked 3D detection with optional predicted trajectories.
#
# Combines a Detection3D (pose, bbox, class, tracking ID) with
# zero or more predicted future paths. Immovable objects such as
# traffic lights may leave the predictions array empty.

# The 3D detection (pose, bounding box, classification, tracking ID)
vision_msgs/Detection3D detection

# Predicted future paths for this object (empty for static objects)
Prediction[] predictions
