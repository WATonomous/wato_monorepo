# ================= Dependencies ===================
# FROM git.uwaterloo.ca:5050/watonomous/wato_carla_api/${CARLA_VERSION}:latest as wato_carla_api


FROM ros:foxy AS base

RUN apt-get update && apt-get install -y curl && \
	rm -rf /var/lib/apt/lists/*

# Add a docker user so that created files in the docker container are owned by a non-root user
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


RUN sudo apt-get update
RUN sudo apt-get install python3-pip -y
RUN curl https://bootstrap.pypa.io/get-pip.py | python3
RUN /home/docker/.local/bin/pip3.8 install carla==${CARLA_VERSION}


# ================= Repositories ===================
FROM base as repo
RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

RUN mkdir -p carla-ros-bridge 


# COPY src/samples/cpp/aggregator aggregator
# COPY src/wato_msgs/sample_msgs sample_msgs
RUN git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# RUN      source /opt/ros/foxy/setup.bash

WORKDIR /home/docker/ament_ws
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
# 	rosdep update && \
# 	rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
# 	colcon build \
# 	--cmake-args -DCMAKE_BUILD_TYPE=Release


COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x ~/wato_ros_entrypoint.sh
# ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]
# CMD source ./install/setup.bash  && ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py

