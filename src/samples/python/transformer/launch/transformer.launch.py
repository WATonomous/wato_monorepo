from launch import LaunchDescription
import launch_ros.actions
import os
import yaml

def generate_launch_description():

    param_file_path = os.path.join(os.path.dirname(__file__), '/home/docker/ament_ws/src/transformer/config/params.yaml')
    with open(param_file_path, 'r') as f:
        params = yaml.safe_load(f)['python_transformer']['ros__parameters']

    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "transformer", package='transformer', executable='transformer', output='screen'),
    ])