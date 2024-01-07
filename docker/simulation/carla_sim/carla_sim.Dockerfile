ARG CARLA_VERSION
FROM carlasim/carla:${CARLA_VERSION} 

ARG CARLA_VERSION
RUN : "${CARLA_VERSION:?Build argument needs to be set and non-empty.}"

USER carla:carla