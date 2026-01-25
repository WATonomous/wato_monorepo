ARG GENERIC_IMAGE

########### Setup WATO Tools and ENV (eg. AMENT_WS, apt-fast) ###########
# This stage can be appended on any publicly available base image to make it ready
# for the wato_monorepo.
FROM ${GENERIC_IMAGE} as wato_base

# use bash + pipefail for all RUN with pipes (DL4006)
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y --no-install-recommends curl sudo && \
    rm -rf /var/lib/apt/lists/*

ENV USER="bolty"
ENV AMENT_WS=/home/${USER}/ament_ws
ENV WATONOMOUS_INSTALL=/opt/watonomous

# Setup WATonomous Install Directory
RUN mkdir -p "${WATONOMOUS_INSTALL}"

# User Setup
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    apt-get install -y --no-install-recommends curl sudo && \
    rm -rf /var/lib/apt/lists/*

# Add a user so that created files in the docker container are owned by a non-root user (for prod)
ARG USER_PASSWD
# hadolint ignore=SC2046
RUN addgroup --gid 1000 ${USER} || groupmod -n ${USER} $(getent group 1000 | cut -d: -f1) && \
    useradd -rm -d /home/${USER} -s /bin/bash -g ${USER} -G sudo -u 1000 ${USER} -p "$(openssl passwd -6 $USER_PASSWD)" || \
    usermod -l ${USER} -d /home/${USER} -m $(getent passwd 1000 | cut -d: -f1) && \
    usermod -aG sudo ${USER} && \
    usermod -p "$(openssl passwd -6 $USER_PASSWD)" ${USER}

# install apt-fast
RUN apt-get update && \
    apt-get install -y --no-install-recommends wget gnupg && \
    mkdir -p /etc/apt/keyrings && \
    wget -qO- "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xA2166B8DE8BDC3367D1901C11EE2FF37CA8DA16B" \
        | gpg --dearmor -o /etc/apt/keyrings/apt-fast.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/apt-fast.gpg] http://ppa.launchpad.net/apt-fast/stable/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/apt-fast.list && \
    apt-get update && \
    echo debconf apt-fast/maxdownloads string 16 | debconf-set-selections && \
    echo debconf apt-fast/dlflag boolean true | debconf-set-selections && \
    echo debconf apt-fast/aptmanager string apt-get | debconf-set-selections && \
    apt-get install -y --no-install-recommends  apt-fast && \
    rm -rf /var/lib/apt/lists/*
