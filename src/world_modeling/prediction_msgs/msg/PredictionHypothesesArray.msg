# PredictionHypothesesArray.msg
# A list of predicted paths per agent, for multi-agent prediction.
#
# Published by Prediction node, consumed by Behaviour Tree and
# Lattice Planner to anticipate other agents' movements and
# plan safe maneuvers.

std_msgs/Header header

# A list of predicted paths per agent. A multi-agent predictor
# might generate a list of possible paths per agent.
PredictionHypotheses[] pred_h_arr
