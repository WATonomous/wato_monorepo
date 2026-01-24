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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Launch behaviour lifecycle node with auto-activation."""
    behaviour_pkg_prefix = get_package_share_directory("behaviour")
    behaviour_param_file = os.path.join(behaviour_pkg_prefix, "config", "params.yaml")

    behaviour_param = DeclareLaunchArgument(
        "behaviour_param_file",
        default_value=behaviour_param_file,
        description="Path to config file for behaviour node",
    )

    config_file = LaunchConfiguration("behaviour_param_file")

    # 1. Create lifecycle node
    behaviour_node = LifecycleNode(
        package="behaviour",
        executable="behaviour_node",
        name="behaviour_node",
        namespace="",
        parameters=[config_file],
        output="screen",
    )

    # 2. Transition to 'configure'
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(behaviour_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # 3. When 'inactive', transition to 'activate'
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=behaviour_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(behaviour_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            behaviour_param,
            behaviour_node,
            configure_event,
            activate_event,
        ]
    )
