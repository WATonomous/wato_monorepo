# ================= Repositories ===================
FROM leungjch/cuda118-tensorrt-base:latest as repo

WORKDIR /home/docker/ament_ws

COPY src/multimodal_object_detection src/multimodal_object_detection
COPY src/wato_msgs/common_msgs src/common_msgs

# Install pcl

RUN sudo apt update 

ENV ROS_PACKAGE_PATH=/opt/ros/humble
# RUN rosinstall_generator perception_pcl --rosdistro ${ROS_DISTRO} --deps --exclude RPP > deps.rosinstall
# RUN cat deps.rosinstall
# RUN vcs import src < deps.rosinstall

RUN sudo ln -s /usr/local/cuda/lib64/libcudart.so /usr/local/lib/
RUN sudo ln -s /usr/local/cuda/lib64/libcudart.a /usr/local/lib/

RUN source src/multimodal_object_detection/src/cuda-bevfusion/tool/environment.sh

RUN export DEBIAN_FRONTEND=noninteractive && . /opt/ros/$ROS_DISTRO/setup.bash && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src \
    --rosdistro $ROS_DISTRO -y \
    --skip-keys "rti-connext-dds-6.0.1" && \
    colcon build \
        --merge-install \
        --packages-select multimodal_object_detection common_msgs pcl_msgs pcl_conversions pcl_ros \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

# Link libbevfusion_core.so to library path
RUN sudo ln -s /home/docker/ament_ws/build/multimodal_object_detection/libbevfusion_core.so /usr/local/lib/libbevfusion_core.so

RUN export LD_LIBRARY_PATH=/home/docker/ament_ws/build/multimodal_object_detection:$LD_LIBRARY_PATH
RUN sudo ldconfig

# Entrypoint will run before any CMD on launch. Sources ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x ~/wato_ros_entrypoint.sh

RUN sudo mkdir -p -m 777 /.ros/log
RUN sudo chmod 777 /home/docker/ament_ws

ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]

CMD ["ros2", "run", "multimodal_object_detection", "multimodal_object_detection"]
# Run with BUILDKIT_PROGRESS=plain ./watod2 up --build