ARG CARLA_VERSION=0.9.13
FROM carlasim/carla:${CARLA_VERSION} AS wato_carla_api

FROM python:3.8.16-slim-bullseye
ARG CARLA_VERSION=0.9.13

RUN pip3 install carla==${CARLA_VERSION} jupyter tensorflow-probability opencv-python numpy open3d
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-glx \
    libegl1-mesa \
    libxrandr2 \
    libxss1 \
    libxcursor1 \
    libxcomposite1 \
    libasound2 \
    libxi6 \
    libxtst6
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y
RUN apt-get update && apt-get install -y vim curl git wget unzip && apt remove python3-networkx

COPY --from=wato_carla_api --chown=root /home/carla/PythonAPI/carla/agents /usr/local/lib/python3.8/site-packages/agents

WORKDIR /home/bolty/carla_data_gen
COPY src/simulation/carla_data_gen /home/bolty/carla_data_gen

WORKDIR /home/bolty

WORKDIR /home/bolty/carla_data_gen

# Set headless CPU rendering for Open3D
RUN export EGL_PLATFORM=surfaceless

ENTRYPOINT ["tail", "-f", "/dev/null"]
