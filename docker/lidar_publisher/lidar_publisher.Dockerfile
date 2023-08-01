# ================= Repositories ===================
FROM leungjch/perception-base:latest as repo

WORKDIR /home/docker/ament_ws

COPY src/lidar_publisher lidar_publisher
COPY src/wato_msgs/common_msgs common_msgs

# RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
#     ros-foxy-ros-base ros-foxy-ackermann-msgs ros-foxy-pcl-conversions ros-foxy-pcl-ros \
#     python3-colcon-common-extensions
ENV ROS_PACKAGE_PATH=/opt/ros/humble
RUN rosinstall_generator perception_pcl --rosdistro ${ROS_DISTRO} --deps --exclude RPP > deps.rosinstall
RUN vcs import src < deps.rosinstall

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

CMD ["ros2", "run", "lidar_publisher", "lidar_publisher"]
# Run with BUILDKIT_PROGRESS=plain ./watod2 up --build