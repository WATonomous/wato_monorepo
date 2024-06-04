# WATonomous Monorepo v2

Dockerized ROS2 setup for the WATonomous Autonomous Vehicle Software Pipeline

- [WATonomous Monorepo](#watonomous-monorepo)
  - [Getting Started](#getting-started)
  - [Description of Files](#description-of-files)
  - [Documentation](#documentation)
    - [Setup Docs](#setup-docs)
    - [Important Topics for Developers](#important-topics-for-developers)
    - [Monorepo Info](#monorepo-info)
    - [Technical Specification](#technical-specification)
    - [FAQ](#faq)

## Getting Started
Read the following:
1. [docs/setup.md](docs/setup/setup.md) How to setup our repo.

**TLDR:** Clone the monorepo, specify active modules, `watod up`. Everything is containerized, so there's little need to setup any dependencies on your end :).

2. [docs/monorepo.md](docs/monorepo.md) What is a monorepo? Why a monorepo?
3. [docs/how_to_dev.md](docs/dev/how_to_dev.md) How to develop in the monorepo.

## Description of Important Files and Directories

Below is a tree diagram of the Monorepo.

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


- `watod`.
  - This is the main bash script that you will use to interface with your containers. More info on `watod`: [docs/dev/watod.md](docs/dev/watod.md).
- `watod-setup-env.sh`.
  - watod-setup-env.sh (in scripts directory) will create a [.env file](https://docs.docker.com/compose/env-file/) specifying environment variables for docker-compose. `watod` automatically runs this script before running any commands. To override variables in `watod-setup-env.sh`, create a `wato2-config.sh` file and populate it with variables, for example `ACTIVE_MODULES="perception path_planning"`. `watod-setup-env.sh` will then take this file into account when building the `.env` file.
- `scripts/watod-completion.bash`.
  - Bash autocomplete for watod. Adapted from docker-compose. Add `source <MONO_DIR>/scripts/watod-completion.bash` to your bashrc to use autocomplete.
- `modules/`:
  - This folder contains all docker-compose files specifying the services we will run. They are grouped up into modules. Note that by default no modules are enabled. To select additional modules, overwrite `ACTIVE_MODULES="<MODULES_OF_YOUR_CHOICE"` in `wato2-config.sh`. See the [docker-compose wiki](https://docs.docker.com/compose/extends/). More info on modules: [docs/dev/modules.md](docs/dev/modules.md).
- `docker/`:
  - This folder contains the `Dockerfiles` for each of our images. [Docker wiki](https://docs.docker.com/engine/reference/builder/).
- `src/`:
  - Here is where all the logic and cool technologies for autonomous software go. The folders in `src` will be mounted to our docker images, so changes in the `src` directory will be reflected in the containers.
- `docs/`:
  - This folder contains the documentation for this monorepo, including instructions for setup and details on how to navigate and develop in this monorepo.

## Documentation

### Setup Docs
[docs/setup](docs/setup)

### Important Topics for Developers
[docs/dev](docs/dev)

### Using Foxglove
[Foxglove](https://foxglove.dev/) is used to visualize ROS messages on a local machine.

Add `data_stream` as an `ACTIVE_MODULES` and `watod up`.

It exposes the port specified by the `FOXGLOVE_BRIDGE_PORT` variable, which you will need to forward to your local machine. This can either be done in the `ports` section of VS Code or by running the command `ssh -L 8765:localhost:8765 <username>@<machine>-ubuntu1.watocluster.local` on your local machine.

Then, open foxglove and add a connection `localhost:8765`, and it should connect.

### Playing Rosbags
Add `infrastructure` as an `ACTIVE_MODULES` and `watod run`.
An example of the command, feel free to change the `mcap` file being run.
`./watod run data_stream ros2 bag play ./nuscenes/NuScenes-v1.0-mini-scene-0061/NuScenes-v1.0-mini-scene-0061_0.mcap`

### Monorepo Info
[docs/monorepo.md](docs/monorepo.md)

### Technical Specification
Docker, Continuous Integration: [tech_spec.md](docs/tech_spec.md)

### FAQ
[docs/faq.md](docs/faq.md)
