# RegulatoryElement.msg
# Generic regulatory element associated with a lanelet.
#
# Replaces hardcoded flags for traffic lights, stop signs, yields, etc.
# Consumers should check the subtype field to identify the element type.

int64 id

# Subtype: "traffic_light", "stop_sign", "yield", "right_of_way", "speed_limit"
string subtype

# Referred primitives (traffic lights, signs, etc.)
Way[] refers

# Reference lines (stop lines) with lanelet association
LaneletWay[] ref_lines

# Yield relationships
int64[] yield_lanelet_ids
int64[] right_of_way_lanelet_ids

# Generic attributes (key-value pairs)
string[] attribute_keys
string[] attribute_values
