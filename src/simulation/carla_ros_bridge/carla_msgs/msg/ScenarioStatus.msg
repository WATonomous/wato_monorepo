std_msgs/Header header

# Name of currently active scenario
string scenario_name

# Human-readable description
string description

# Scenario state: idle, loading, starting, running, paused, error
string state

# Additional information (e.g., error messages)
string info

# Authoritative active bundle, also used by late-starting consumers.
string map_id
string osm_map_path
string projector_type
float64 origin_lat
float64 origin_lon
bool trajectories_enabled
# Incremented for every load, including reloading the same scenario.
uint64 generation
