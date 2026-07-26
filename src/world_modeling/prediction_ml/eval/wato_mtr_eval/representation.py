# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""The WATO information contract.

Rung 1 (Python ablation) and rung 2 (C++ SceneBuilder) must implement this
identically, or the delta between them is meaningless. This module is the
machine-readable half; REPRESENTATION.md is the human half.

Pure: imports neither rosbag2_py nor torch, so it loads in both environments.
"""

# Tensor contract (mirrors config/contract_8_target.yaml).
TARGET_CAPACITY = 8
CONTEXT_CAPACITY = 128
HISTORY_STEPS = 11
OBJ_FEATURE_DIM = 29
NUM_SRC_POLYLINES = 768
POINTS_PER_POLYLINE = 20
MAP_FEATURE_DIM = 9
NUM_MODES = 6
NUM_FUTURE_FRAMES = 80
TRAJ_CHANNELS = 7

# Timing.
HISTORY_RATE_HZ = 10.0
MODEL_STEP_S = 0.1
DEPLOYED_HORIZON_S = 3.0
DEPLOYED_STEP_S = 0.2
DEPLOYED_STEPS = 15
NATIVE_HORIZON_S = 8.0

# obj_trajs column layout (scene_builder.cpp:763-781 in the design snapshot).
OBJ_COL = {
    "position": slice(0, 3),
    "size": slice(3, 6),
    "type_onehot": slice(6, 9),
    "is_center": slice(9, 10),
    "is_sdc": slice(10, 11),
    "time_onehot": slice(11, 22),
    "scalar_time": slice(22, 23),
    "heading": slice(23, 25),
    "velocity": slice(25, 27),
    "acceleration": slice(27, 29),
}

# map_polylines column layout (scene_builder.cpp:939 in the design snapshot).
MAP_COL = {
    "position": slice(0, 3),
    "direction": slice(3, 6),
    "global_type": slice(6, 7),
    "pre_position": slice(7, 9),
}

MTR_TYPE_CODES = {"vehicle": 0, "pedestrian": 1, "cyclist": 2}
CLASS_ID_FOR_TYPE = {
    "vehicle": "vehicle",
    "pedestrian": "pedestrian",
    "cyclist": "cyclist",
}
VELOCITY_CLASS_ID = "linear_velocity"

DISCARDED = {
    "map_polyline_types": "~20 Waymo types collapsed to one lane-centerline token",
    "road_edges": "present in Lanelet.msg as BOUNDARY_ROAD_EDGE; not packed",
    "crosswalks": "present in Lanelet.msg as lanelet_type=='crosswalk'; not packed",
    "lane_boundaries": "left/right boundary geometry present; not packed",
    "traffic_light_state": (
        "genuinely absent: no dynamic signal state in lanelet_msgs"
    ),
    "stop_lines": "present as RegulatoryElement.ref_lines; not packed",
}

WATO_GLOBAL_TYPE = 0
