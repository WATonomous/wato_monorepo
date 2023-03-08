FROM nvcr.io/nvidia/tensorrt:23.01-py3 as base

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y curl vim wget build-essential manpages-dev wget zlib1g software-properties-common git && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get --reinstall install sudo

RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

ENV ROS_DISTRO foxy
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base ros-foxy-ackermann-msgs ros-foxy-pcl-conversions ros-foxy-pcl-ros \
    python3-colcon-common-extensions
RUN sudo apt-get install -y --no-install-recommends python3-rosdep && sudo rosdep init

# Add a docker user so we that created files in the docker container are owned by a non-root user
RUN addgroup --gid 1000 docker && \
    adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# Remap the docker user and group to be the same uid and group as the host user.
# Any created files by the docker container will be owned by the host user.
RUN USER=docker && \
    GROUP=docker && \                                                                     
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \                                                                                                            
    chown root:root /usr/local/bin/fixuid && \                                                                              
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \                                                                                               
    printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/docker/" > /etc/fixuid/config.yml

USER docker:docker

RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

# ================= Repositories ===================
FROM base as repo

COPY src/lidar_object_detection lidar_object_detection
COPY src/wato_msgs/common_msgs common_msgs

WORKDIR /home/docker/ament_ws
RUN wget --header='PRIVATE-TOKEN:glpat-ZyqY_SDbwsUbcs4tjn1z' 'https://gitlab.uwaterloo.ca/api/v4/projects/18367/repository/files/pp%2Fpp_og.onnx/raw?ref=add_pp' -O "pointpillar.onnx"
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

# Entrypoint will run before any CMD on launch. Sources ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x ~/wato_ros_entrypoint.sh
ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]
CMD ["ros2", "run", "lidar_object_detection"]