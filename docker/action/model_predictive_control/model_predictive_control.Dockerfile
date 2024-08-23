FROM python:3.8.16-slim-bullseye
ARG CARLA_VERSION=0.9.13

# Install system dependencies and pip packages
RUN apt-get update && apt-get install -y \
    curl git wget unzip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install CARLA and other Python packages
RUN pip3 install carla==${CARLA_VERSION} tensorflow-probability numpy

# Install CasADi
RUN pip3 install casadi

WORKDIR /home/bolty/carla_notebooks
COPY src/action/model_predictive_control /home/bolty/carla_notebooks

WORKDIR /home/bolty/carla_notebooks
