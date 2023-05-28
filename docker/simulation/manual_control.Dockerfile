# ARG CARLA_VERSION
# FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

# FROM ros:foxy-ros-core AS api

# ARG CARLA_VERSION
# RUN apt-get update && \
#       apt-get install -y lsb-release \
#       software-properties-common \
#       build-essential \
#       wget \
#       curl \
#       python3-rosdep \
#       python3-rospkg \
#       ros-foxy-tf2-geometry-msgs \
#       ros-foxy-tf2-eigen \
#       git \
#       python3-colcon-common-extensions \
#       python3-pygame \
#       supervisor

# #Install Python Carla API
# COPY --from=wato_carla_api --chown=root /home/carla/PythonAPI/carla /opt/carla/PythonAPI
# WORKDIR /opt/carla/PythonAPI
# RUN python3.8 -m easy_install pip && \
#       pip3 install carla==${CARLA_VERSION}

# # Add a docker user so we that created files in the docker container are owned by a non-root user
# RUN addgroup --gid 1000 docker && \
#       adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
#       echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# # Remap the docker user and group to be the same uid and group as the host user.
# # Any created files by the docker container will be owned by the host user.
# RUN USER=docker && \
#       GROUP=docker && \
#       curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
#       chown root:root /usr/local/bin/fixuid && \
#       chmod 4755 /usr/local/bin/fixuid && \
#       mkdir -p /etc/fixuid && \
#       printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/docker/" > /etc/fixuid/config.yml

# USER docker:docker

# ENV DEBIAN_FRONTEND noninteractive
# RUN sudo chsh -s /bin/bash
# ENV SHELL=/bin/bash

# # ================= Repositories ===================

# FROM api as base
# # Set up workspace
# RUN mkdir -p ~/ament_ws/src
# WORKDIR /home/docker/ament_ws/src

# RUN sudo apt-get install -y git
# RUN git clone --depth 1 --branch master https://github.com/carla-simulator/ros-bridge.git
# WORKDIR /home/docker/ament_ws/src/ros-bridge
# RUN git submodule update --init --recursive
# WORKDIR /home/docker/ament_ws/src

# # Download rviz 
# RUN git clone --branch foxy https://github.com/ros2/rviz.git

# # Download source dependencies

# FROM base as repo
# COPY src/wato_msgs ros_msgs
# # COPY src/simulation/carla_config carla_config

# RUN sudo rm /etc/ros/rosdep/sources.list.d/20-default.list || true && \
#       sudo rosdep init && \
#       sudo apt-get update && \
#       rosdep update && \
#       rosdep install --from-paths . --ignore-src -r -y

# WORKDIR /home/docker/ament_ws

# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash; colcon build"

# RUN rm -rf build

# # Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/catkin_ws/devel/setup.bash
# COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
# RUN sudo chmod +x ~/wato_ros_entrypoint.sh

# ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]
# #CMD ["/bin/bash"]

# # ================= Develop ===================
# FROM repo as debug

# USER root:root

# RUN apt-get update -y && apt-get install -y wget curl gdb supervisor
# EXPOSE 5900
# RUN apt-get update && apt-get install -y lxde x11vnc xvfb mesa-utils && apt-get purge -y light-locker

# COPY --chown=docker docker/simulation/supervisord.conf /etc/supervisor/supervisord.conf
# RUN chown -R docker:docker /etc/supervisor

# RUN chmod 777 /var/log/supervisor/
# ENV DISPLAY=:1.0 
# CMD ["/usr/bin/supervisord"]




# ================= Dependencies ===================
FROM ros:foxy-ros-core AS base

RUN apt-get update && apt-get install -y curl && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    # Install ROS viz tools
    ros-foxy-rqt* \
    ros-foxy-rviz2 \
    # misc
    wget curl tmux \
    lsb-release \
    software-properties-common \
    build-essential \
    python3-rosdep \
    python3-rospkg \
    ros-foxy-tf2-geometry-msgs \
    ros-foxy-tf2-eigen \
    git pip \
    python3-colcon-common-extensions \
    python3-pygame

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

ENV DEBIAN_FRONTEND noninteractive
RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

# Install carla ros bridge for their GUI tools
RUN git clone --depth 1 --branch master https://github.com/carla-simulator/ros-bridge.git
WORKDIR /home/docker/ament_ws/src/ros-bridge
RUN git submodule update --init --recursive

# COPY src/wato_msgs/sample_msgs sample_msgs
# If needed, you can copy over rviz configs here

USER root:root

WORKDIR /home/docker/ament_ws
RUN . /opt/ros/foxy/setup.sh && \
    rosdep init && \
    rosdep update && \
    rosdep -i install --from-paths . --ignore-src -y && \
    colcon build

USER docker:docker

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
ENTRYPOINT ["/usr/local/bin/fixuid", "-q", "/home/docker/wato_ros_entrypoint.sh"]

# ================= Develop ===================
FROM repo as debug

USER root:root

RUN apt-get update -y && apt-get install -y wget curl gdb supervisor
EXPOSE 5900
RUN apt-get update && apt-get install -y lxde x11vnc xvfb mesa-utils && apt-get purge -y light-locker

COPY --chown=docker docker/simulation/supervisord.conf /etc/supervisor/supervisord.conf
RUN chown -R docker:docker /etc/supervisor

RUN chmod 777 /var/log/supervisor/
ENV DISPLAY=:1.0 
CMD ["/usr/bin/supervisord"]