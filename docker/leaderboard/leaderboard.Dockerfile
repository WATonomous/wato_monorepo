FROM ros:foxy as base

RUN apt-get update && apt-get install -y wget tar

WORKDIR /home/docker/carla
RUN wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/Leaderboard/CARLA_Leaderboard_20.tar.gz && \
    tar -xf CARLA_Leaderboard_20.tar.gz

# # RUN apt-get update && apt-get install -y \
# #     libomp5 \
# #     curl \
# #     xdg-user-dirs \
# #     xdg-utils \
# #     python3.7 \
# #     python3-pip \
# #     libgeos-dev \
# #     mesa-utils \
# #     x11vnc \
# #     xvfb
# # RUN pip install gdown

# RUN apt-get update && apt-get install -y python3-pip libomp5 libgeos-dev curl && pip3 install gdown
# RUN cd Import && gdown --id 1H3aCrg41js9mL5mmgDRfgyylJ8t3Vf0E
# RUN ./ImportAssets.sh

RUN apt-get update && apt-get install -y python3-pip curl firefox

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

WORKDIR /home/docker
RUN git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/leaderboard.git
RUN git clone -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/scenario_runner.git
RUN git clone --recurse-submodules -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/ros-bridge

RUN cd carla && pip3 install -r PythonAPI/carla/requirements.txt
RUN cd leaderboard && pip3 install -r requirements.txt
RUN cd scenario_runner && pip3 install -r requirements.txt

# WORKDIR /home/docker/carla

# # COPY src/leaderboard/.bashrc /home/docker/

# # COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
# # COPY docker/.bashrc /home/docker/.bashrc
# # RUN sudo chmod +x ~/wato_ros_entrypoint.sh
# # ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]

USER docker:docker

ENTRYPOINT ["/usr/local/bin/fixuid", "-q"]

# # CMD ["./CarlaUE4.sh"]

# FROM carlasim/carla:0.9.11

# USER root:root

# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
# RUN apt update && apt install -y python3-pip && pip3 install gdown
# RUN cd /home/carla/Import && gdown --id 1H3aCrg41js9mL5mmgDRfgyylJ8t3Vf0E
# RUN cd /home/carla && ./ImportAssets.sh

# USER carla:carla

# FROM carlasim/carla:0.9.10.1 

# USER root:root

# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
# RUN apt update && apt upgrade -y && apt install -y python3-pip && pip3 install gdown
# COPY docker/leaderboard/import_maps.sh import_maps.sh
# RUN ./import_maps.sh

# USER carla:carla

# USER root:root

# ENV DEBIAN_FRONTEND noninteractive
# RUN apt-get update && \
#     apt-get install -y lxde x11vnc xvfb mesa-utils supervisor && \
#     apt-get purge -y light-locker

# EXPOSE 5900

# COPY --chown=docker docker/gui_tools/supervisord.conf /etc/supervisor/supervisord.conf
# RUN chown -R docker:docker /etc/supervisor
# RUN chmod 777 /var/log/supervisor/

# USER docker:docker

# ENV DISPLAY=:1.0
# RUN sudo chsh -s /bin/bash
# ENV SHELL=/bin/bash

# CMD ["/usr/bin/supervisord"]

