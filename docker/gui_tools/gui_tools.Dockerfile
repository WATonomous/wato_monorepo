FROM ros:foxy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y curl wget python3-pip libgeos-dev && pip3 install pygame

RUN apt-get update && \
    apt-get install -y lxde x11vnc xvfb mesa-utils supervisor && \
    apt-get purge -y light-locker

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

RUN mkdir -p /home/docker/src
WORKDIR /home/docker/src
RUN git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/leaderboard.git
RUN git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/scenario_runner.git
RUN git clone --recurse-submodules -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/ros-bridge.git

WORKDIR /home/docker/src/carla
RUN wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/Leaderboard/CARLA_Leaderboard_20.tar.gz && \
    tar -xf CARLA_Leaderboard_20.tar.gz
RUN pip3 install -r PythonAPI/carla/requirements.txt

WORKDIR /home/docker/src/leaderboard
RUN pip3 install -r requirements.txt

WORKDIR /home/docker/src/scenario_runner
RUN pip3 install -r requirements.txt

WORKDIR /home/docker/src/ros-bridge
RUN . /opt/ros/foxy/setup.sh && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build

USER root:root

EXPOSE 5900
EXPOSE 2000

WORKDIR /home/docker/src

COPY --chown=docker docker/gui_tools/supervisord.conf /etc/supervisor/supervisord.conf
RUN chown -R docker:docker /etc/supervisor
RUN chmod 777 /var/log/supervisor/

COPY docker/wato_ros_entrypoint.sh /home/docker
COPY docker/gui_tools/.bashrc /home/docker

USER docker:docker
ENV DISPLAY=:1.0

ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]
CMD ["/usr/bin/supervisord"]
