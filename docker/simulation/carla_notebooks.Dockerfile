ARG CARLA_VERSION
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

FROM python:3.8.16-slim-bullseye
ARG CARLA_VERSION

#Install Python Carla API
COPY --from=wato_carla_api --chown=root /home/carla/PythonAPI/carla /opt/carla/PythonAPI
WORKDIR /opt/carla/PythonAPI
RUN pip3 install carla==${CARLA_VERSION}
RUN pip3 install jupyter

WORKDIR /root/carla_notebooks
COPY --chown=root src/simulation/carla_notebooks /root/carla_notebooks/
