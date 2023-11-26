ARG GENERIC_BASE

########### Setup WATO Tools and ENV (eg. AMENT_WS, apt-fast) ###########
# This stage can be appended on any publicly available base image to make it ready
# for the wato_monorepo.
FROM ${GENERIC_BASE} as wato_base

ENV AMENT_WS=/home/docker/ament_ws
ENV DEBIAN_FRONTEND noninteractive
ENV USER="docker"

# fix user permissions when deving in container
COPY docker/fixuid_setup.sh /tmp/fixuid_setup.sh
RUN /tmp/fixuid_setup.sh

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
