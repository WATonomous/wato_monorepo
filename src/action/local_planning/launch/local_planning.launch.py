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
# Copyright 2023 WATonomous
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

import launch
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart

import launch_ros
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition

import lifecycle_msgs.msg


def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory("local_planning"), "config", "params.yaml"
    )

    local_planner_node = LifecycleNode(
        package="local_planning",
        executable="local_planning_node",
        name="local_planning_node",
        namespace="",
        parameters=[param_file_path],
        output="screen",
    )

    # Configure node after it inits
    # TODO REMOVE THESE TWO AND USE LIFECYCLE MANAGER
    configure_on_start = RegisterEventHandler(
        OnProcessStart(
            target_action=local_planner_node,
            on_start=[
                EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(local_planner_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    # Activate node after it is configured
    # NOTE this will turn the node active everytime you deactivate it 
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=local_planner_node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(local_planner_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            local_planner_node,
            configure_on_start,
            # activate_on_inactive,
        ]
    )
