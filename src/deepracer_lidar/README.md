```
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select deepracer_lidar
. install/setup.bash
ros2 run deepracer_lidar listener
colcon build --symlink-install --packages-select deepracer_lidar
```