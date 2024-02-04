ARG CARLA_VERSION=0.9.13
FROM carlasim/carla:${CARLA_VERSION} 

ARG CARLA_VERSION=0.9.13
RUN : "${CARLA_VERSION:?Build argument needs to be set and non-empty.}"

USER carla:carla