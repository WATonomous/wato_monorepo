#!/bin/sh

rosinstall_generator ros2_socketcan > ros2_socketcan.rosinstall
rosinstall_generator can_msgs > can_msgs.rosinstall
rosinstall . ros2_socketcan.rosinstall
rosinstall . can_msgs.rosinstall
