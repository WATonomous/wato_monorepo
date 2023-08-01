# ================= Repositories ===================
FROM leungjch/perception-base as repo

WORKDIR /home/docker/ament_ws

COPY src/lidar_object_detection/pointpillar.onnx pointpillar.onnx
COPY src/lidar_object_detection src/lidar_object_detection
COPY src/wato_msgs/common_msgs src/common_msgs

# Install pcl
ENV ROS_PACKAGE_PATH=/opt/ros/humble
RUN rosinstall_generator perception_pcl --rosdistro ${ROS_DISTRO} --deps --exclude RPP > deps.rosinstall
RUN cat deps.rosinstall
RUN vcs import src < deps.rosinstall

RUN export DEBIAN_FRONTEND=noninteractive && . /opt/ros/$ROS_DISTRO/setup.bash && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src \
    --rosdistro $ROS_DISTRO -y \
    --skip-keys "rti-connext-dds-6.0.1" && \
    colcon build \
        --merge-install \
        --packages-select lidar_object_detection common_msgs pcl_msgs pcl_conversions pcl_ros \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

# Entrypoint will run before any CMD on launch. Sources ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x ~/wato_ros_entrypoint.sh

RUN sudo mkdir -p -m 777 /.ros/log
RUN sudo chmod 777 /home/docker/ament_ws

ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]

CMD ["ros2", "run", "lidar_object_detection", "lidar_object_detection"]
# Run with BUILDKIT_PROGRESS=plain ./watod2 up --build