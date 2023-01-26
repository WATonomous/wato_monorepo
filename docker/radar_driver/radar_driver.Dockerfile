FROM ros:foxy AS base

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-kinetic-perception-pcl build-essential \
    python-catkin-tools python-rosdep libpcap0.8-dev \
    wget curl

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

RUN mkdir -p /home/docker/catkin_ws/src
WORKDIR /home/docker/catkin_ws

COPY src/sensor_interfacing/radar_driver src/radar_driver

RUN catkin config --extend /opt/ros/kinetic && \
    catkin build && \
    rm -rf .catkin_tools build

COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc

RUN sudo chmod +x ~/wato_ros_entrypoint.sh

ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]

CMD ["roslaunch", "--wait", "radar_driver", "radarROSbag.launch"]
