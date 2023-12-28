ARG GENERIC_IMAGE

########### Setup WATO Tools and ENV (eg. AMENT_WS, apt-fast) ###########
# This stage can be appended on any publicly available base image to make it ready
# for the wato_monorepo.
FROM ${GENERIC_IMAGE} as wato_base

ENV AMENT_WS=/home/docker/ament_ws
ENV DEBIAN_FRONTEND noninteractive
ENV USER="docker"

# User Setup
RUN apt-get update && apt-get install -y curl sudo && \
    rm -rf /var/lib/apt/lists/*

# Add a docker user so that created files in the docker container are owned by a non-root user
RUN addgroup --gid 1000 docker && \
    adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# install apt-fast
RUN apt-get -qq update && \
    apt-get install -qq -y wget && \
    mkdir -p /etc/apt/keyrings && \
    wget -O - "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xA2166B8DE8BDC3367D1901C11EE2FF37CA8DA16B" \
        | gpg --dearmor -o /etc/apt/keyrings/apt-fast.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/apt-fast.gpg] \
        http://ppa.launchpad.net/apt-fast/stable/ubuntu focal main" > /etc/apt/sources.list.d/apt-fast.list && \
    apt-get update -qq && apt-get install -qq -y apt-fast && \
    echo debconf apt-fast/maxdownloads string 16 | debconf-set-selections && \
    echo debconf apt-fast/dlflag boolean true | debconf-set-selections && \
    echo debconf apt-fast/aptmanager string apt-get | debconf-set-selections
