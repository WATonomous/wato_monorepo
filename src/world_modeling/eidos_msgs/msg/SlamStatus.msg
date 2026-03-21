std_msgs/Header header

# SLAM state enum
uint8 INITIALIZING = 0
uint8 RELOCALIZING = 1
uint8 TRACKING = 2
uint8 state 0

# Current state index in the pose graph
int32 current_state_index 0

# Number of keyframes in the map
int32 num_keyframes 0

# Number of factors in the pose graph
int32 num_factors 0

# Whether a loop closure was recently detected
bool loop_closure_detected false

# Active factor plugins
string[] active_factor_plugins

# Active relocalization plugins
string[] active_relocalization_plugins
