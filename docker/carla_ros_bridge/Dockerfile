# ================= Dependencies ===================
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

# ================= Repositories ===================
FROM base as repo
RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

RUN mkdir -p carla-ros-bridge 

# COPY src/samples/cpp/aggregator aggregator
# COPY src/wato_msgs/sample_msgs sample_msgs

WORKDIR /home/docker/ament_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
	rosdep update && \
	rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
	colcon build \
	--cmake-args -DCMAKE_BUILD_TYPE=Release



