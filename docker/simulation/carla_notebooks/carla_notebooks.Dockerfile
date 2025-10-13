ARG CARLA_VERSION=0.9.13
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

FROM python:3.10.19-slim-bookworm
ARG CARLA_VERSION=0.9.15

RUN pip3 install --no-cache-dir \
        carla==${CARLA_VERSION} \
        jupyter==1.0.0 \
        tensorflow-probability==0.23.0

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
RUN pip3 install --no-cache-dir -r requirements.txt && \
    sed -i 's/hero/ego/g' srunner/tools/scenario_parser.py

WORKDIR /home/bolty/carla_notebooks
COPY src/simulation/carla_notebooks /home/bolty/carla_notebooks
