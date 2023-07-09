ARG CARLA_VERSION
FROM carlasim/carla:${CARLA_VERSION} 

ARG CARLA_VERSION
RUN : "${CARLA_VERSION:?Build argument needs to be set and non-empty.}"

COPY src/simulation/carla_notebooks/maps /home/carla/CarlaUE4/Content/Carla/Maps

USER carla:carla
