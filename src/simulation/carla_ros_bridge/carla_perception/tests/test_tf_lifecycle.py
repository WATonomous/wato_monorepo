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
"""Exercise TF history and subscriptions across CARLA scenario changes."""

from unittest.mock import MagicMock

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.time import Time

from carla_perception import bbox_publisher_node, lidar_publisher_node


@pytest.fixture(params=["bbox", "lidar"])
def perception_node(request, monkeypatch):
    rclpy.init()
    module, node_class = {
        "bbox": (bbox_publisher_node, bbox_publisher_node.BBoxPublisherNode),
        "lidar": (lidar_publisher_node, lidar_publisher_node.LidarPublisherNode),
    }[request.param]
    client = MagicMock()
    client.get_world.return_value.get_settings.return_value.fixed_delta_seconds = 1 / 60
    monkeypatch.setattr(module, "connect_carla", lambda *args: client)
    monkeypatch.setattr(module, "find_ego_vehicle", lambda *args: MagicMock())
    node = node_class(node_name=f"test_{request.param}_tf")
    try:
        node.set_parameters([Parameter("use_sim_time", value=True)])
        if request.param == "lidar":
            node.set_parameters(
                [Parameter("lidar_names", Parameter.Type.STRING_ARRAY, [])]
            )
        yield node
    finally:
        if node.tf_buffer is not None:
            node.trigger_cleanup()
        node.destroy_node()
        rclpy.shutdown()


def _transform(seconds):
    transform = TransformStamped()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    transform.header.stamp = Time(seconds=seconds).to_msg()
    transform.transform.rotation.w = 1.0
    return transform


def test_world_clock_reset_accepts_new_episode_transforms(perception_node):
    node = perception_node
    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    node.get_clock().set_ros_time_override(Time(seconds=100))
    node.tf_buffer.set_transform(_transform(100), "test")
    assert node.tf_buffer.can_transform("odom", "base_footprint", Time())

    node.get_clock().set_ros_time_override(Time(seconds=1))
    assert not node.tf_buffer.can_transform("odom", "base_footprint", Time())
    node.tf_buffer.set_transform(_transform(1), "test")

    transform = node.tf_buffer.lookup_transform("odom", "base_footprint", Time())
    assert transform.header.stamp.sec == 1


def test_scenario_cleanup_releases_tf_subscriptions_and_history(perception_node):
    node = perception_node
    subscriptions_before = len(list(node.subscriptions))
    services_before = len(list(node.services))

    for _ in range(2):
        assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        assert len(list(node.subscriptions)) == subscriptions_before + 2
        node.tf_buffer.set_transform(_transform(100), "test")
        assert node.trigger_cleanup() == TransitionCallbackReturn.SUCCESS
        assert len(list(node.subscriptions)) == subscriptions_before
        assert len(list(node.services)) == services_before
        assert node.tf_listener is None
        assert node.tf_buffer is None

    assert node.trigger_configure() == TransitionCallbackReturn.SUCCESS
    node.tf_buffer.set_transform(_transform(1), "test")
    transform = node.tf_buffer.lookup_transform("odom", "base_footprint", Time())
    assert transform.header.stamp.sec == 1
