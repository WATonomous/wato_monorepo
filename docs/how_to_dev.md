# How to Develop in the Monorepo

Below is a tree diageram of the Monorepo.

```
wato_monorepo_v2
├── dev_config.sh
├── docker
│   ├── samples
│   │   └── cpp
│   │       ├── Dockerfile.aggregator
│   │       ├── Dockerfile.producer
│   │       └── Dockerfile.transformer
│   └── wato_ros_entrypoint.sh
├── docs
├── profiles
│   └── docker-compose.samples.yaml
├── scripts
│   ├── format_files.sh
│   └── watod2-completion.bash
├── src
│   ├── motion_planning_and_control
│   ├── perception
│   ├── ros_msgs
│   │   └── sample_msgs
│   │       ├── CMakeLists.txt
│   │       ├── msg
│   │       └── package.xml
│   ├── samples
│   │   └── cpp
│   │       ├── aggregator
│   │       ├── image
│   │       ├── producer
│   │       ├── README.md
│   │       └── transformer
│   ├── sensor_interfacing
│   ├── simulation
│   ├── tools
│   └── world_modeling
└── watod2
```

## How to code in the Monorepo
Say you are trying to create a new ROS2 node. 

1. What is the node's functionality?
2. What subgroup of ASD is responsible for that ROS2 node?
3. What is the input/output of that node?

Once those are decided. Find where your code fits in this repo. 

## 1.0 src
---

### 1.1 Where does my ROS2 node go in the src
Based on the subgroup, your ROS2 node should exist inside the subgroup's folder within the `src` directory
* eg. if you are part of the `motion planning and control` subteam, and your ROS2 node is for `motion planning and control`, then your ROS2 node should exist under the directory `/src/motion_planning_and_control/<your_node>/`

### 1.2 Where do my custom ROS2 message go?
The ROS2 package creating your custom message should go under `/wato_messages`, similar structure to how you add a ROS2 node into the rest of the src. 
* from the last example, a custom ROS2 message for your node in `motion planning and control` should go under `/src/wato_msgs/motion_planning_and_control/<your_node>/`

### How do I init a ROS2 node to begin development on? 
We don't have ROS2 downloaded in the Server machines, therefore, just copy one of the sample nodes (depending on what language you plan on using)
* **I want to made a cpp ROS2 node:** copy `/src/samples/cpp/aggregator/` to the directory you decided on based on 1.1
* **I want to made a python ROS2 node:** copy `/src/samples/python/aggregator/` to the directory you decided on based on 1.1

### How do I init a ROS2 message to begin development on?
We don't have ROS2 downloaded in the Server machines, therefore, just copy one of the sample nodes (depending on what language you plan on using)
* **I want to made a cpp ROS2 node:** copy `/src/wato_msgs/samples/cpp/aggregator/` to the directory you decided on based on 1.2
* **I want to made a python ROS2 node:** copy `/src/wato_msgs/samples/python/aggregator/` to the directory you decided on based on 1.2

## 2.0 docker and profiles
---

### 2.1 premise
Docker and Docker Compose allow for modular development with little friction between ROS2 nodes. What this means is, the dependencies of a single ROS2 node in the monorepo is GUARENTEED to not clash with the dependencies of other ROS2 nodes. This is powerful because it allows us to rapidly remove and integrate new algorithms seemlessly without having to deal with clashing dependencies.

### 2.2 Docker Compose and Profiles
The profiles directory contains Docker Compose files that specify which nodes should startup. 

## 2.2.1 Do I need to create my own profile (docker compose file)?
Contact your lead. Usually your ROS2 node will not need its own profile. It is up to your lead to decide whether your node should startup with a profile of their choosing. 

## 2.2.2 Structure of the docker compose file 
A typical docker compose file consists of the following...
```
version: "3.8"
services:
  aggregator: # name of the service
    build:
      context: ..
      # path to the dockerfile in the monorepo that this service needs to up with
      dockerfile: docker/samples/cpp/aggregator.Dockerfile 
      # helps with caching from the registry (dw about this for now) (boilerplate, but with name change)
      cache_from:
        - "${SAMPLES_CPP_AGGREGATOR_IMAGE:?}-${CACHE_FROM_TAG}"
        - "${SAMPLES_CPP_AGGREGATOR_IMAGE:?}-develop"
    # name of the image made by the dockerfile (boilerplate, but with name change)
    image: "${SAMPLES_CPP_AGGREGATOR_IMAGE:?}-${TAG}"
    # deals with permission and ownership in the container (boilerplate)
    user: ${FIXUID:?}:${FIXGID:?}
    # IMPORTANT: mounts your ROS2 node into the container so that changes in the dockerfile are reflected in your 
    # source code
    # https://docs.docker.com/storage/volumes/
    volumes:
      - ../src/samples/cpp/aggregator:/home/docker/ament_ws/src/aggregator
  producer:
  .
  .
  .
```

The extent to which you will change this file is most likely to create a new service under `services`.

### 2.3 Your ROS2 node's dockerfile
A dockerfile specifies how to setup an environment (you could say a small isolated computer) that purely just runs your ROS2 node. The language used in the dockerfile is the same as the commands you use in a Linux terminal except with `RUN` in front of it. (as well as some other nice-to-haves like `ENV` which sets environment variables)

A typical dockerfile consists of the following...
```
# ================= Dependencies ===================
FROM ros:humble AS base # we use the ROS2 Humble base image, what this means is, your little vm already has ROS2 installed
ENV DEBIAN_FRONTEND noninteractive # makes it so that any install is non-interactive and your build won't stall

RUN apt-get update && apt-get install -y curl && \
    rm -rf /var/lib/apt/lists/*

# ADD OTHER DEPENDENCIES FOR YOUR ROS2 NODE HERE (pip, apt (apt-get), possibly other repos if they need to be built)

# Add a docker user so that created files in the docker container are owned by a non-root user (BOILERPLATE)
RUN addgroup --gid 1000 docker && \
    adduser --uid 1000 --ingroup docker --home /home/docker --shell /bin/bash --disabled-password --gecos "" docker && \
    echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

# Remap the docker user and group to be the same uid and group as the host user. (BOILERPLATE)
# Any created files by the docker container will be owned by the host user.
RUN USER=docker && \
    GROUP=docker && \                                                                     
    curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \                                                                                                            
    chown root:root /usr/local/bin/fixuid && \                                                                              
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \                                                                                               
    printf "user: $USER\ngroup: $GROUP\npaths:\n  - /home/docker/" > /etc/fixuid/config.yml

USER docker:docker

RUN sudo chsh -s /bin/bash
ENV SHELL=/bin/bash

# ================= Repositories ===================
FROM base as repo # build stage, you can specify to what stage you want to build the dockerfile to

# CREATE ROS2 WORKSPACE
RUN mkdir -p ~/ament_ws/src
WORKDIR /home/docker/ament_ws/src

# COPY IN YOUR ROS2 NODE CODE (note that copy and volume will overlap each other on purpose)
COPY src/samples/cpp/aggregator aggregator
COPY src/wato_msgs/sample_msgs sample_msgs

# BUILD YOUR ROS2 NODE
WORKDIR /home/docker/ament_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# SOURCE THE CODE AND DEAL WITH FILE PERMISSIONS AND OWNERSHIP USING OUR ENTRYPOINT SCRIPT (half boilerplate, need to 
# specify CMD to roslaunch your code)
# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh /home/docker/wato_ros_entrypoint.sh
COPY docker/.bashrc /home/docker/.bashrc
RUN sudo chmod +x ~/wato_ros_entrypoint.sh
ENTRYPOINT ["/home/docker/wato_ros_entrypoint.sh"]
CMD ["ros2", "launch", "aggregator", "aggregator.launch.py"]
```

A dockerfile can be very finnicy at times. So if you have any questions, please ask infra.

## 3.0 Linking up your ROS2 Node and docker stuff to watod2
---
The `watod2` script is a custom built script to setup environment variables (some of which are very important for file permissions and managing images in the server) and up certain profiles. `watod2` is power because it gives us the freedom to pick different profiles (docker compose files) to run in a single, easy-to-use script.

### 3.1 watod2
This script does not change regardless of any changes. Only people editing this script is Infra.

### 3.2 dev_config.sh
This script is ran by `watod2` to setup all of the environment variables. YOU NEED TO ADD SOME NEW ENVIRONEMENT OF YOUR OWN IN ORDER FOR YOUR CODE TO WORK WITH `watod2`. In the `dev_config.sh` there are 2 sections with which you need to add things.

#### 3.2.1 Images
Under the `images` section of the `dev_config.sh`...

```
## --------------------------- Images -------------------------------

# ROS2 C++ Samples
SAMPLES_CPP_AGGREGATOR_IMAGE=${SAMPLES_CPP_AGGREGATOR_IMAGE:-"git.uwaterloo.ca:5050/watonomous/wato_monorepo/samples_cpp_aggregator"}
```
And under `environment variables` ...
```
## -------------------- Environment Variables -------------------------

echo "# Auto-generated by ${BASH_SOURCE[0]}. Please do not edit." > "$PROFILES_DIR/.env"
echo "ACTIVE_PROFILES=\"$ACTIVE_PROFILES\"" > "$PROFILES_DIR/.env"
echo "PROFILE_BLACKLIST=\"$PROFILE_BLACKLIST\"" >> "$PROFILES_DIR/.env"

echo "COMPOSE_DOCKER_CLI_BUILD=1" >> "$PROFILES_DIR/.env"
echo "COMPOSE_PROJECT_NAME=$COMPOSE_PROJECT_NAME" >> "$PROFILES_DIR/.env"

echo "ROS_IP=$ROS_IP" >> "$PROFILES_DIR/.env"
echo "ROS_HOSTNAME=$ROS_HOSTNAME" >> "$PROFILES_DIR/.env"

# ADD YOUR IMAGE NAME HERE AND ECHO INTO .ENV
echo "SAMPLES_CPP_AGGREGATOR_IMAGE=$SAMPLES_CPP_AGGREGATOR_IMAGE" >> "$PROFILES_DIR/.env"
```

This is for image caching, which you saw in the docker compose file of section 2.2.2

