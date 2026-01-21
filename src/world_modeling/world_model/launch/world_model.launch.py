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
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = get_package_share_directory('world_model')
    default_config = os.path.join(pkg_share, 'config', 'world_model.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the world_model configuration file'
    )

    osm_map_path_arg = DeclareLaunchArgument(
        'osm_map_path',
        default_value='',
        description='Path to the OSM lanelet map file'
    )

    world_model_node = LifecycleNode(
        package='world_model',
        executable='world_model_node',
        name='world_model',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'osm_map_path': LaunchConfiguration('osm_map_path'),
            }
        ],
    )

    # Automatically configure the node after it starts
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(world_model_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Automatically activate after configure succeeds
    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=world_model_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(world_model_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        config_file_arg,
        osm_map_path_arg,
        world_model_node,
        configure_event,
        activate_event_handler,
    ])
