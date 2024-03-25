ARG CARLA_VERSION=0.9.13
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

FROM python:3.8.16-slim-bullseye
ARG CARLA_VERSION=0.9.13

RUN pip3 install carla==${CARLA_VERSION} jupyter tensorflow-probability opencv-python numpy
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y
RUN apt-get update && apt-get install -y vim curl git wget unzip && apt remove python3-networkx

COPY --from=wato_carla_api --chown=root /home/carla/PythonAPI/carla/agents /usr/local/lib/python3.8/site-packages/agents

WORKDIR /home/bolty/carla_data_gen
COPY src/simulation/carla_data_gen /home/bolty/carla_data_gen

WORKDIR /home/bolty

WORKDIR /home/bolty/carla_data_gen

ENTRYPOINT ["tail", "-f", "/dev/null"]
