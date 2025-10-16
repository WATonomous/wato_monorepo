# WATonomous Monorepo (for EVE)

Dockerized monorepo for the WATonomous autonomous vehicle project (dubbed EVE).

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. Our monorepo infrastructure supports Linux Ubuntu >= 22.04, Windows (WSL/WSL2), and MacOS. Though, aspects of this repo might require specific hardware like NVidia GPUs.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository). If you are using WSL, install docker outside of WSL, it will automatically setup docker within WSL for you.
3. You're all set! Information on running the monorepo with our infrastructure is given [here](https://wiki.watonomous.ca/autonomous_software_general/monorepo_infrastructure/)

## Available Modules

### Infrastructure

Starts the foxglove bridge and data streamer for rosbags.

```
wato_monorepo_v2
├── watod-setup-env.sh
├── docker
│   ├── samples
│   │   └── cpp
│   │       ├── Dockerfile.aggregator
│   │       ├── Dockerfile.producer
│   │       └── Dockerfile.transformer
│   └── wato_ros_entrypoint.sh
├── docs
├── modules
│   └── docker-compose.samples.yaml
├── scripts
├── src
│   ├── motion_planning_and_control
│   ├── perception
|   ├── prediction
│   ├── wato_msgs
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
└── watod
```

### Interfacing

Launches packages directly connecting to hardware. This includes the sensors of the car and the car itself. [see docs](src/interfacing/INTERFACING_README.md)

### Perception

Launches packages for perception. [see docs](src/perception/PERCEPTION_README.md)

### World Modeling

Launches packages for world modeling. [see docs](src/world_modeling/WM_README.md)

### Action

Launches packages for action. [see docs](src/action/ACTION_README.md)

### Simulation

Launches packages CARLA simulator. [see docs](src/simulation/CARLA_README.md)

## Contribute

Information on contributing to the monorepo is given in [DEVELOPING.md](./DEVELOPING.md)
