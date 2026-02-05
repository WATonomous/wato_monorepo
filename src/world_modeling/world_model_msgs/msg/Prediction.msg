# Prediction.msg
# Defines a predicted path with a given confidence score.
#
# Represents a single trajectory hypothesis for an agent.
# Published by Prediction node, consumed by Behaviour Tree and
# Lattice Planner to anticipate other agents' movements.

std_msgs/Header header

# Confidence score for this prediction (0.0 to 1.0)
float64 conf

# Predicted path as a sequence of timestamped poses
geometry_msgs/PoseStamped[] poses
