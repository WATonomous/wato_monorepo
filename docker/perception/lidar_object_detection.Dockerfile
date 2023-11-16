FROM leungjch/cuda118-tensorrt-base:latest as base


#### Setup rest of the environment

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

# Add a docker user so we that created files in the docker container are owned by a non-root user
# RUN addgroup --gid 1000 docker && \
    # adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    # echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# Remap the docker user and group to be the same uid and group as the host user.
# Any created files by the docker container will be owned by the host user.
# RUN USER=docker && \
#     GROUP=docker && \                                                                     
#     curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \                                                                                                            
#     chown root:root /usr/local/bin/fixuid && \                                                                              
#     chmod 4755 /usr/local/bin/fixuid && \
#     mkdir -p /etc/fixuid && \                                                                                               
#     printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/docker/" > /etc/fixuid/config.yml


# # setup entrypoint
# COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh
# COPY scripts/variables.sh /variables.sh



# # Set permissions
# RUN sudo chmod +x /ros_entrypoint.sh
# RUN sudo chmod -R 777 /home/docker
# RUN sudo chmod -R 777 /opt/ros



WORKDIR /home/docker/ament_ws
COPY src/perception/lidar_object_detection/model /model/pointpillars_model
RUN nvidia-smi >&2

RUN ROS_DISTRO=humble

# # convert the pointpillars model from etlt format to tensorrt engine   
# RUN /model/pointpillars_model/tao-converter  -k tlt_encode \
#                -e /model/pointpillars_model/trt.engine \
#                -p points,1x204800x4,1x204800x4,1x204800x4 \
#                -p num_points,1,1,1 \
#                -t fp16 \
#                /model/pointpillars_model/pointpillars_deployable.etlt

COPY src/perception/lidar_object_detection pointpillars_ws

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

# Entrypoint will run before any CMD on launch. Sources ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x /home/docker/wato_ros_entrypoint.sh

RUN sudo mkdir -p -m 777 /.ros/log
RUN sudo chmod 777 /home/docker/ament_ws
RUN sudo chmod -R 777 /root

USER docker:docker


ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]

CMD ["ros2", "launch", "pp_infer", "pp_infer_launch.py"]
# Must run with DOCKER_BUILDKIT=0 ./watod up --build
# Buildkit does not support gpus as of Sept17 https://github.com/docker/compose/issues/9681