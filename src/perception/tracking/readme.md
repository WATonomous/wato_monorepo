watod build tracking

watod up tracking foxglove data_stream

watod -t tracking 
colcon build
ros2 launch tracking tracking.launch.py

watod run data_stream ros2 bag play ./nuscenes/NuScenes-v1.0-mini-scene-0061/NuScenes-v1.0-mini-scene-0061_0.mcap --loop