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
"""The representation contract must agree with the frozen tensor contract."""

import pathlib
import re
import subprocess
import sys

from wato_mtr_eval import representation as rep

CONTRACT = (
    pathlib.Path(__file__).resolve().parents[2] / "config" / "contract_8_target.yaml"
)


def test_obj_columns_partition_the_feature_dim():
    """Every one of the 29 columns is claimed exactly once, with no gaps."""
    claimed = []
    for column_slice in rep.OBJ_COL.values():
        claimed.extend(range(column_slice.start, column_slice.stop))
    assert sorted(claimed) == list(range(rep.OBJ_FEATURE_DIM))


def test_deployed_steps_derive_from_horizon_and_step():
    assert rep.DEPLOYED_STEPS == int(rep.DEPLOYED_HORIZON_S / rep.DEPLOYED_STEP_S)


def test_deployed_step_is_whole_multiple_of_model_step():
    ratio = rep.DEPLOYED_STEP_S / rep.MODEL_STEP_S
    assert abs(ratio - round(ratio)) < 1e-9


def test_shapes_match_the_frozen_contract_yaml():
    text = CONTRACT.read_text()

    def shape_of(name: str) -> list[int]:
        match = re.search(
            rf"- name: {name}\s+dtype: \w+\s+shape: \[([^\]]+)\]",
            text,
        )
        assert match, f"{name} not found in contract"
        return [int(value) for value in match.group(1).split(",")]

    assert shape_of("obj_trajs") == [
        rep.TARGET_CAPACITY,
        rep.CONTEXT_CAPACITY,
        rep.HISTORY_STEPS,
        rep.OBJ_FEATURE_DIM,
    ]
    assert shape_of("map_polylines") == [
        rep.TARGET_CAPACITY,
        rep.NUM_SRC_POLYLINES,
        rep.POINTS_PER_POLYLINE,
        rep.MAP_FEATURE_DIM,
    ]
    assert shape_of("pred_trajs") == [
        rep.TARGET_CAPACITY,
        rep.NUM_MODES,
        rep.NUM_FUTURE_FRAMES,
        rep.TRAJ_CHANNELS,
    ]


def test_module_does_not_import_ros_or_torch():
    code = """
import sys
import wato_mtr_eval.representation
assert "rosbag2_py" not in sys.modules
assert "torch" not in sys.modules
"""
    subprocess.run([sys.executable, "-c", code], check=True)
