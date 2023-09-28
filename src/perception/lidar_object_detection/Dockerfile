FROM leungjch/cuda118-tensorrt-base:latest as base

WORKDIR /home/docker/ament_ws

COPY model /model/pointpillars_model

RUN ROS_DISTRO=humble

# convert the etlt to tensorrt engine   
RUN /model/pointpillars_model/tao-converter  -k tlt_encode \
               -e /model/pointpillars_model/trt.engine \
               -p points,1x204800x4,1x204800x4,1x204800x4 \
               -p num_points,1,1,1 \
               -t fp16 \
               /model/pointpillars_model/pointpillars_deployable.etlt

COPY lidar_object_detection pointpillars_ws

RUN export DEBIAN_FRONTEND=noninteractive && . /opt/ros/$ROS_DISTRO/setup.bash && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src \
    --rosdistro $ROS_DISTRO -y \
    --skip-keys "rti-connext-dds-6.0.1" && \
    colcon build \
        --merge-install \
        --packages-select pp_infer \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

COPY wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY .bashrc /home/docker/.bashrc
RUN sudo chmod +x ~/wato_ros_entrypoint.sh

RUN sudo mkdir -p -m 777 /.ros/log
RUN sudo chmod 777 /home/docker/ament_ws

ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]

CMD ["ros2", "launch", "pp_infer", "pp_infer_launch.py"]