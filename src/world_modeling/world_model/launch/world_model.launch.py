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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


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

    lat_origin_arg = DeclareLaunchArgument(
        'lat_origin',
        default_value='0.0',
        description='Latitude origin for UTM projection'
    )

    lon_origin_arg = DeclareLaunchArgument(
        'lon_origin',
        default_value='0.0',
        description='Longitude origin for UTM projection'
    )

    world_model_node = Node(
        package='world_model',
        executable='world_model_node',
        name='world_model',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'osm_map_path': LaunchConfiguration('osm_map_path'),
                'lat_origin': LaunchConfiguration('lat_origin'),
                'lon_origin': LaunchConfiguration('lon_origin'),
            }
        ],
    )

    return LaunchDescription([
        config_file_arg,
        osm_map_path_arg,
        lat_origin_arg,
        lon_origin_arg,
        world_model_node,
    ])
