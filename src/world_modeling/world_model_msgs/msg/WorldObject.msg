# WorldObject.msg
# A tracked 3D detection with lanelet context, history, and predicted trajectories.
#
# Unified message for all tracked entities. Produced by the prediction node
# (with only detection + predictions populated) and enriched by the world model
# (which fills in header, lanelet_ahead, and history).

std_msgs/Header header

# The 3D detection (pose, bbox, classification, tracking ID)
vision_msgs/Detection3D detection

# Lanelet context: current lanelet + reachable lanelets ahead (empty from perception)
lanelet_msgs/LaneletAhead lanelet_ahead

# Historical path (most recent first; empty from perception, populated by world model)
geometry_msgs/PoseStamped[] history

# Predicted future paths
Prediction[] predictions
