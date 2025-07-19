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

# For creating launch arguments
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# To load params from yaml file
import os
import yaml
from ament_index_python.packages import get_package_share_directory

# For using other launch files
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Use the second param (the launch argument) unless it is empty


def CheckParam(param1, param2):
    try:
        return param2
    except NameError:
        return param1


def generate_launch_description():
    """Get file path of yaml file"""
    config_file_path = os.path.join(
        get_package_share_directory("carla_config"), "config", "carla_settings.yaml"
    )
    # mpc_bridge_confile_file_path = os.path.join(
    #     get_package_share_directory("carla_config"), "config", "mpc_bridge_config.yaml"
    # )

    """ Load params from yaml file """
    with open(config_file_path, "r") as config_file:
        params = yaml.safe_load(config_file)
    # with open(mpc_bridge_confile_file_path, "r") as config_file:
    #     mpc_bridge_config = yaml.safe_load(config_file)

    """ Get hostname from yaml file """
    # Checking if the hostname provided in the yaml file is referencing an env
    # variable
    if "$" in params["carla"]["host"]:
        hostname = os.environ.get(
            params["carla"]["host"].split()[1], params["carla"]["host"].split()[2]
        )
    else:
        hostname = params["carla"]["host"]

    """ Declare launch arguments """
    # carla-ros-bridge args: See carla_settings.yaml for default arg values
    host_arg = DeclareLaunchArgument("host", default_value=hostname)
    port_arg = DeclareLaunchArgument("port", default_value=str(params["carla"]["port"]))
    timeout_arg = DeclareLaunchArgument(
        "timeout", default_value=str(params["carla"]["timeout"])
    )
    sync_mode_arg = DeclareLaunchArgument(
        "synchronous_mode", default_value=str(params["carla"]["synchronous_mode"])
    )
    sync_mode_wait_arg = DeclareLaunchArgument(
        "synchronous_mode_wait_for_vehicle_control_command",
        default_value=str(
            params["carla"]["synchronous_mode_wait_for_vehicle_control_command"]
        ),
    )
    fixed_delta_arg = DeclareLaunchArgument(
        "fixed_delta_seconds", default_value=str(params["carla"]["fixed_delta_seconds"])
    )
    town_arg = DeclareLaunchArgument("town", default_value=params["carla"]["town"])

    # Carla ego vehicle args
    objects_definition_arg = DeclareLaunchArgument(
        "objects_definition_file",
        default_value=str(
            get_package_share_directory("carla_config") + "/config/objects.json"
        ),
    )
    role_name_arg = DeclareLaunchArgument(
        "role_name", default_value=params["carla"]["ego_vehicle"]["role_name"]
    )
    spawn_point_arg = DeclareLaunchArgument("spawn_point", default_value="None")
    spawn_sensors_only_arg = DeclareLaunchArgument(
        "spawn_sensors_only", default_value="False"
    )

    # Ackermann control args
    control_loop_rate_arg = DeclareLaunchArgument(
        "control_loop_rate",
        default_value=str(params["carla"]["ackermann_control"]["control_loop_rate"]),
    )

    """Launch carla ros bridge node."""
    carla_ros_bridge = Node(
        package="carla_ros_bridge",
        executable="bridge",
        output="screen",
        parameters=[
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "timeout": LaunchConfiguration("timeout"),
                "synchronous_mode": LaunchConfiguration("synchronous_mode"),
                "synchronous_mode_wait_for_vehicle_control_command": LaunchConfiguration(
                    "synchronous_mode_wait_for_vehicle_control_command"
                ),
                "fixed_delta_seconds": LaunchConfiguration("fixed_delta_seconds"),
                "town": LaunchConfiguration("town"),
                "ego_vehicle_role_name": LaunchConfiguration("role_name"),
            }
        ],
        respawn=True,
    )

    """Launch ego vehicle spawner nodes (using carla_example_ego_vehicle.launch.py)"""
    carla_ego_vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("carla_spawn_objects"),
                "carla_example_ego_vehicle.launch.py",
            )
        ),
        launch_arguments={
            "objects_definition_file": LaunchConfiguration("objects_definition_file"),
            "role_name": LaunchConfiguration("role_name"),
            "spawn_point_ego_vehicle": LaunchConfiguration("spawn_point"),
            "spawn_sensors_only": LaunchConfiguration("spawn_sensors_only"),
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "timeout": LaunchConfiguration("timeout"),
        }.items(),
    )

    carla_control = []
    if os.environ.get("USE_ACKERMANN_CONTROL", "False").lower() == "True":
        """ Launch Ackermann Control Node """
        carla_control = carla_control.append(
            Node(
                package="carla_ackermann_control",
                executable="carla_ackermann_control_node",
                output="screen",
                parameters=[
                    {
                        "role_name": LaunchConfiguration("role_name"),
                        "control_loop_rate": LaunchConfiguration("control_loop_rate"),
                    }
                ],
            )
        )

    """ Launch MPC Bridge Node """
    # carla_mpc_bridge = Node(
    #     package="carla_config",
    #     executable="carla_mpc_bridge",
    #     parameters=[
    #         {
    #             "mpc_moutput_topic": mpc_bridge_config["mpc_bridge_node"][
    #                 "ros_parameters"
    #             ]["mpc_output_topic"],
    #             "steering_publisher_topic": mpc_bridge_config["mpc_bridge_node"][
    #                 "ros_parameters"
    #             ]["steering_publisher_topic"],
    #         }
    #     ],
    #     output="screen",
    # )

    waypoint_topic = DeclareLaunchArgument(
        "waypoint_topic",
        default_value=["/carla/", LaunchConfiguration("role_name"), "/waypoints"],
    )
    waypoint_topic_old = DeclareLaunchArgument(
        "waypoint_topic_old",
        default_value=["/carla/", LaunchConfiguration("role_name"), "/waypointsOld"],
    )

    """ Launch CARLA Waypoint Publisher """
    carla_waypoint_publisher = Node(
        package="carla_waypoint_publisher",
        executable="carla_waypoint_publisher",
        name="carla_waypoint_publisher",
        output="screen",
        parameters=[
            {
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "timeout": LaunchConfiguration("timeout"),
                "role_name": LaunchConfiguration("role_name"),
            }
        ],
        remappings=[
            (
                LaunchConfiguration("waypoint_topic"),
                LaunchConfiguration("waypoint_topic_old"),
            ),
        ],
    )

    """ Launch Waypoint Modifier Node """
    carla_waypoint_modifier = Node(
        package="carla_config",
        executable="carla_waypoint_modifier",
        parameters=[
            {
                "input_topic": LaunchConfiguration("waypoint_topic_old"),
                "output_topic": LaunchConfiguration("waypoint_topic"),
            }
        ],
        output="screen",
    )

    """ Launch Waypoint Modifier Node """
    carla_image_encoding_conversion = Node(
        package="carla_config",
        executable="carla_image_encoding_conversion",
        output="screen",
    )

    return LaunchDescription(
        [
            host_arg,
            port_arg,
            timeout_arg,
            sync_mode_arg,
            sync_mode_wait_arg,
            fixed_delta_arg,
            town_arg,
            objects_definition_arg,
            role_name_arg,
            spawn_point_arg,
            spawn_sensors_only_arg,
            control_loop_rate_arg,
            carla_ros_bridge,
            carla_ego_vehicle,
            *carla_control,
            waypoint_topic_old,
            waypoint_topic,
            carla_waypoint_publisher,
            carla_waypoint_modifier,
            carla_image_encoding_conversion,
            # carla_mpc_bridge, # MPC bridge needs to be reworked
        ]
    )
