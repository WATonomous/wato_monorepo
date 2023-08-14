# ================= Repositories ===================
FROM leungjch/perception-ubuntu2004-base:latest as repo

WORKDIR /home/docker/ament_ws

COPY src/sensor_fusion_publisher sensor_fusion_publisher
COPY src/wato_msgs/common_msgs common_msgs

# To run rosbags
RUN sudo apt-get update && sudo apt-get install -y curl ros-humble-ros2bag ros-humble-rosbag2* ros-humble-foxglove-msgs&& \
    sudo rm -rf /var/lib/apt/lists/*

RUN export DEBIAN_FRONTEND=noninteractive && . /opt/ros/$ROS_DISTRO/setup.bash && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build

# Entrypoint will run before any CMD on launch. Sources ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x ~/wato_ros_entrypoint.sh

RUN sudo mkdir -p -m 777 /.ros/log
RUN sudo chmod 777 /home/docker/ament_ws

ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]

CMD ["ros2", "run", "sensor_fusion_publisher", "sensor_fusion_publisher"]