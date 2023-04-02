from launch import LaunchDescription
import launch_ros.actions
import os
import yaml

def generate_launch_description():

    param_file_path = os.path.join(os.path.dirname(__file__), '/home/docker/ament_ws/src/producer/config/params.yaml')
    with open(param_file_path, 'r') as f:
        params = yaml.safe_load(f)['python_producer']['ros__parameters']

    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "producer", package='producer', executable='producer', output='screen'),
    ])