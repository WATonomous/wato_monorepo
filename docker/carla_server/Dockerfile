ARG CARLA_VERSION
FROM carlasim/carla:${CARLA_VERSION} 

ARG CARLA_VERSION
RUN : "${CARLA_VERSION:?Build argument needs to be set and non-empty.}"

USER root:root

# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
RUN apt update && apt upgrade -y && apt-get install -y python3-pip xvfb && pip3 install gdown

# COPY import_maps.sh import_maps.sh
# RUN ./import_maps.sh

USER carla:carla
