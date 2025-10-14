ARG CARLA_VERSION=0.9.13
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

FROM python:3.8.16-slim-bullseye
ARG CARLA_VERSION=0.9.13


RUN python3 -m pip install --no-cache-dir --upgrade \
    pip==24.2 setuptools==70.0.0 wheel==0.44.0 && \
    pip3 install --no-cache-dir \
    carla==${CARLA_VERSION} \
    jupyter==1.0.0 \
    keras==2.13.1


RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl \
        git \
        wget \
        unzip && \
    apt-get remove -y python3-networkx && \
    rm -rf /var/lib/apt/lists/*

COPY --from=wato_carla_api --chown=root \
    /home/carla/PythonAPI/carla/agents \
    /usr/local/lib/python3.8/site-packages/agents

WORKDIR /home/bolty

RUN git clone https://github.com/carla-simulator/scenario_runner.git

WORKDIR /home/bolty/scenario_runner

# CHECKOUT the tag that matches your CARLA version:
RUN git fetch --tags && git checkout v0.9.13

RUN pip3 install --no-cache-dir -r requirements.txt && \
    sed -i 's/hero/ego/g' srunner/tools/scenario_parser.py

WORKDIR /home/bolty/carla_notebooks
COPY src/simulation/carla_notebooks /home/bolty/carla_notebooks
