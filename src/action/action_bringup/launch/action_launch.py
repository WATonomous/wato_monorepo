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
    local_planning_pkg_dir = get_package_share_directory('local_planning')
    local_planning_params_file = os.path.join(local_planning_pkg_dir, 'config', 'local_planner_params.yaml')
    
    return LaunchDescription(
        [
            Node(
                package="model_predictive_control",
                executable="mpc_node",
                name="mpc_node",
                output="screen",
            ),
            Node(
                package="local_planning",
                executable="local_planner_node",
                name="local_planner_node",
                namespace='action/local_planning',
                parameters=[local_planning_params_file],
                output="screen"
            )
        ]
    )
