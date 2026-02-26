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


from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    freeroam_planner_params = os.path.join(
        get_package_share_directory("freeroam_planner"), "config", "params.yaml"
    )
    pure_pursuit_params = os.path.join(
        get_package_share_directory("ackermann_pure_pursuit"), "config", "params.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="model_predictive_control",
                executable="mpc_node",
                name="mpc_node",
                output="screen",
            ),
            Node(
                package="freeroam_planner",
                executable="freeroam_planner_node",
                name="freeroam_planner_node",
                output="screen",
                parameters=[freeroam_planner_params],
            ),
            Node(
                package="ackermann_pure_pursuit",
                executable="pure_pursuit_node",
                name="pure_pursuit_node",
                output="screen",
                parameters=[pure_pursuit_params],
            ),
            Node(
                package="wato_lifecycle_manager",
                executable="wato_lifecycle_manager_node",
                name="action_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "node_names": [
                            "freeroam_planner_node",
                            "pure_pursuit_node",
                        ],
                        "autostart": True,
                        "transition_timeout_s": 10.0,
                        "bond_timeout_s": 4.0,
                        "bond_enabled": True,
                    }
                ],
            ),
        ]
    )
