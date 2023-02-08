FROM ros:humble AS base

RUN apt-get update && apt-get install -y curl && \
    rm -rf /var/lib/apt/lists/*

RUN addgroup --gid 1000 docker && \
    adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

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

FROM base as repo

RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

COPY src/sensor_interfacing/continental_driver continental_driver

WORKDIR /home/docker/ament_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
ENTRYPOINT ["/usr/local/bin/fixuid", "-q", "/home/docker/wato_ros_entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]
# CMD ["ros2", "launch", "continental_driver", "continental_driver.launch.py"]
