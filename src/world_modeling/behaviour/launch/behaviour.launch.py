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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


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

    # When the node reaches 'inactive' (after configuring), transition to 'activate'
    # configure_event = RegisterEventHandler(
    #     OnStateTransition(
    #         target_lifecycle_node=behaviour_node,
    #         goal_state="inactive",
    #         entities=[
    #             EmitEvent(
    #                 event=ChangeState(
    #                     lifecycle_node_matcher=matches_action(behaviour_node),
    #                     transition_id=Transition.TRANSITION_ACTIVATE,
    #                 )
    #             )
    #         ],
    #     )
    # )

    # # Emit configure event on startup to move from 'unconfigured' to 'inactive'
    # startup_event = EmitEvent(
    #     event=ChangeState(
    #         lifecycle_node_matcher=matches_action(behaviour_node),
    #         transition_id=Transition.TRANSITION_CONFIGURE,
    #     )
    # )

    return LaunchDescription(
        [
            behaviour_param,
            behaviour_node,
            # configure_event,
            # startup_event,
        ]
    )
