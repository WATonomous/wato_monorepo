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
"""No module may import both the ROS and PyTorch stacks."""

import ast
import pathlib

import pytest

PKG = pathlib.Path(__file__).resolve().parents[1] / "wato_mtr_eval"

ROS_IMPORTS = {
    "rosbag2_py",
    "rclpy",
    "vision_msgs",
    "geometry_msgs",
    "nav_msgs",
    "lanelet_msgs",
}
MODEL_IMPORTS = {"torch", "tensorrt", "onnx", "onnxruntime"}


def _imported_roots(path: pathlib.Path) -> set[str]:
    tree = ast.parse(path.read_text(), filename=str(path))
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            roots.add(node.module.split(".")[0])
    return roots


@pytest.mark.parametrize("path", sorted(PKG.glob("*.py")), ids=lambda p: p.name)
def test_no_module_mixes_ros_and_model_stacks(path):
    roots = _imported_roots(path)
    ros_hits = roots & ROS_IMPORTS
    model_hits = roots & MODEL_IMPORTS
    assert not (ros_hits and model_hits), (
        f"{path.name} imports both ROS {sorted(ros_hits)} and model "
        f"{sorted(model_hits)}. Split it: the two environments are separate installs."
    )
