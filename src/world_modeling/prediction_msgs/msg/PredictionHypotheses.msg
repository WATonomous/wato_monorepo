# PredictionHypotheses.msg
# Defines a list of possible paths a single agent could take.
#
# Contains all prediction hypotheses for a single agent,
# identified by a unique ID.

std_msgs/Header header

# The unique numeric ID of the agent (matches tracking ID)
int64 id

# Path predictions for this agent (multiple hypotheses)
Prediction[] preds
