# ExecuteBehaviour.srv
# Sends lattice planner the message to execute the accurate behaviour and
# the preferred lanelets that the car should be on.

string behaviour
int64[] preferred_lanelet_ids