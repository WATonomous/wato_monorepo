ARG CARLA_VERSION
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

FROM python:3.8.16-slim-bullseye
ARG CARLA_VERSION

RUN pip3 install carla==${CARLA_VERSION} jupyter tensorflow-probability
RUN apt-get update && apt-get install -y curl git wget unzip && apt remove python3-networkx

COPY --from=wato_carla_api --chown=root /home/carla/PythonAPI/carla/agents /usr/local/lib/python3.8/site-packages/agents

# fix user permissions when deving in container
COPY docker/fixuid_setup.sh /project/fixuid_setup.sh
RUN /project/fixuid_setup.sh
USER docker:docker

ENTRYPOINT ["fixuid", "-q"]

WORKDIR /home/docker/carla_notebooks
COPY src/simulation/carla_notebooks /home/docker/carla_notebooks

WORKDIR /home/docker
# Setup CARLA Scenario Runner
# The last sed command replaces hero (default ego vehicle name) with another ego vehicle name
RUN git clone https://github.com/carla-simulator/scenario_runner.git && \
    cd scenario_runner && pip3 install -r requirements.txt && \
    sed -i s/np.int,/int,/g /home/docker/.local/lib/python3.8/site-packages/networkx/readwrite/graphml.py && \
    sed -i s/hero/ego/g /home/docker/scenario_runner/srunner/tools/scenario_parser.py
WORKDIR /home/docker
