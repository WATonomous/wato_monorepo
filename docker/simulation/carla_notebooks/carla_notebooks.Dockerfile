ARG CARLA_VERSION=0.9.13
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

FROM python:3.8.16-slim-bullseye
ARG CARLA_VERSION=0.9.13

RUN pip3 install carla==${CARLA_VERSION} jupyter tensorflow-probability
RUN apt-get update && apt-get install -y curl git wget unzip && apt remove python3-networkx

COPY --from=wato_carla_api --chown=root /home/carla/PythonAPI/carla/agents /usr/local/lib/python3.8/site-packages/agents

WORKDIR /home/bolty/carla_notebooks
COPY src/simulation/carla_notebooks /home/bolty/carla_notebooks

WORKDIR /home/bolty
# Setup CARLA Scenario Runner
# The last sed command replaces hero (default ego vehicle name) with another ego vehicle name
RUN git clone https://github.com/carla-simulator/scenario_runner.git && \
    cd scenario_runner && pip3 install -r requirements.txt && \
    sed -i s/hero/ego/g /home/bolty/scenario_runner/srunner/tools/scenario_parser.py
WORKDIR /home/bolty

WORKDIR /home/bolty/carla_notebooks