# WATonomous Monorepo v2

Dockerized ROS2 setup for the WATonomous Autonomous Driving Software Pipeline

- [WATonomous Monorepo](#watonomous-monorepo)
  - [Getting Started](#getting-started)
  - [Description of Files](#description-of-files)
  - [Profiles](#profiles)
  - [Common Operations](#common-operations)
    - [watod2](#watod2)
    - [Remote VScode](#remote-vscode)
    - [Playing ros2 bags](#playing-ros2-bags)
    - [Using Foxglove](#using-foxglove)
    - [Using Carla Jupyter Notebook](#using-carla-jupyter-notebook)
  - [Monorepo Info](#monorepo-info)
  - [Technical Specification](#technical-specification)
  - [Testing Infrastructure](#testing)
  - [FAQ](#faq)
    - [Every time I make a new branch I need to rebuild all the images from scratch](#every-time-i-make-a-new-branch-i-need-to-rebuild-all-the-images-from-scratch)
    - ["Invalid reference format" when using watod2](#invalid-reference-format-when-using-watod2)

## Getting Started
Read the following:
1. [docs/setup.md](docs/setup/setup.md) How to setup our repo. 

**TLDR:** Clone the monorepo, specify active profiles, `watod2 up`. Everything is containerized, so there's little need to setup any dependencies on your end :).

2. [docs/monorepo.md](docs/monorepo.md) What is a monorepo? Why a monorepo?
3. [docs/how_to_dev.md](docs/dev/how_to_dev.md) How to develop in the monorepo.

## Description of Important Files and Directories

Below is a tree diagram of the Monorepo.

```
wato_monorepo_v2
├── watod2-setup-env.sh
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
└── watod2
```


- `watod2`. 
  - This is the main bash script that you will use to interface with your containers. More info on `watod2`: [docs/dev/watod2.md](docs/dev/watod2.md).
- `watod2-setup-env.sh`. 
  - watod2-setup-env.sh (in scripts directory) will create a [.env file](https://docs.docker.com/compose/env-file/) specifying environment variables for docker-compose. `watod2` automatically runs this script before running any commands. To override variables in `watod2-setup-env.sh`, create a `wato2-config.sh` file and populate it with variables, for example `ACTIVE_PROFILES="perception path_planning"`. `watod2-setup-env.sh` will then take this file into account when building the `.env` file.
- `scripts/watod2-completion.bash`.
  - Bash autocomplete for watod2. Adapted from docker-compose. Add `source <MONO_DIR>/scripts/watod2-completion.bash` to your bashrc to use autocomplete.
- `profiles/`: 
  - This folder contains all docker-compose files specifying the services we will run. They are grouped up into profiles. Note that by default no profiles are enabled. To select additional profiles, overwrite `ACTIVE_PROFILES="<PROFILES_OF_YOUR_CHOICE"` in `wato2-config.sh`. See the [docker-compose wiki](https://docs.docker.com/compose/extends/). More info on profiles: [docs/dev/profiles.md](docs/dev/profiles.md).
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

**Building images**: `watod2 build`

**Seeing exposed ports**: `watod2 --ports`
- Your docker containers expose a certain number of applications that can be accessed publicly. For example, [VNC](https://en.wikipedia.org/wiki/Virtual_Network_Computing)
- Start your containers with `watod2 up` then in another terminal use `watod2 --ports`
- `watod2 -lp` will  also print information if you want to forward the ports from the external server to your local machine over SSH.
- `watod2 -lc [local_base_port]` will generate a [~/.ssh/config file](https://linuxize.com/post/using-the-ssh-config-file/) for you to copy to your local machine. A newly generated config file will always be up to date with VM hostnames, watod2 ports forwards, etc... and in our oppinion is the best way to configure your communication with WATO's server cluster. The optional `[local_base_port]` argument allows you to define your own port range on your local machine. For example, if the `pp_env_model` service is running on remote port `3864`, with your remote base port being `3860`, then running `watod2 -lc 1000` will place a forwarding rule in the config to forward the  `pp_env_model`'s code server to `localhost:1004`.
  
**Opening a shell inside a docker container**: `watod2 -t <SERVICE_NAME>`
- Opens a bash shell into the specified service. Find a list of your services using `watod2 ps --services`
- From here, you can execute commands inside the docker container. For example, ROS2 commands. 

### Remote VScode

**Over SSH**

1. Download and install [Visual Studio Code](https://code.visualstudio.com/) on your local machine
3. In VS Code, install the [Remote Development extension pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)
4. Select the "Remote Explorer" tab in the left taskbar
5. Make sure the Remote Explorer is targeting "SSH Targets" and click plus to add the server you are trying to access.  For example, "wato-tr.uwaterloo.ca".
6. Right click on the added target and connect. Select your workspace, and you can edit code in `src`.
7. To make an integrated VScode terminal that runs on the host, use `ctrl->p` `> Create New Integrated Terminal`.

**Over SSH and Docker**

If you want to attach VScode to a specific container instead of to the host, you need to follow some extra steps as the interaction between vscode-remote and vscode-docker is buggy. The issue is currently being tracked here: https://github.com/microsoft/vscode-remote-release/issues/2514

1. Download and install [Docker](https://www.docker.com) on your local machine
2. Follow the "Over SSH" instructions
3. Once your VScode is connected to the server, install the [Docker extension pack](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) on the remote VScode
4. in VScode, open `settings.json` (`ctrl-p` `>settings.json`)
5. Add the line: `"docker.host": "tcp://localhost:23750",` near the top
6. In a terminal on your local machine, start an SSH tunnel for the docker daemon
    `ssh -L localhost:23750:/var/run/docker.sock user@hostname.`
7. Enter the VS Code Docker extension tab and find your container in the "Individual Containers" section.
8. Right click on your container and select "Attach VS Code to Container"
9. Done! At this point you should be able to edit code and run Catkin/ROS commands in the container through the VS Code terminal

### Playing ROS2 Bags

A bag is a file that stores serialized ROS2 message data. We can play a bag to make data available on various topics.
More on bags can be found here: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html.

Add `data_stream` as an `ACTIVE_PROFILE` in `dev_config.local.sh`. 

Run `watod2 up` (or however you want to launch the `data_stream` service). 

The working directory of the `data_stream` container should have a `nuscenes` directory, which contains the NuScenes dataset converted to ros2bag format. To confirm this, run `watod run data_stream ls nuscenes` to view the available bags. Each bag has its own directory. The location of the `.mcap` file is `<name>/<name>_0.mcap`. For example, one of the bags is in `nuscenes/NuScenes-v1.0-mini-scene-0061/NuScenes-v1.0-mini-scene-0061_0.mcap`. 

Now, using `watod run data_stream [ROS2 BAG COMMAND]` you can run any `ros2 bag ...` command as documented here: http://wiki.ros.org/rosbag/Commandline. You probably want to explore `ros2 bag play ...`: http://wiki.ros.org/rosbag/Commandline#rosbag_play. (Since this documentation is in ROS1, you can replace `rosbag` with `ros2 bag` to run the equivalent command in ROS2)

Example: `watod2 run data_stream ros2 bag play ./nuscenes/NuScenes-v1.0-mini-scene-0061/NuScenes-v1.0-mini-scene-0061_0.mcap`

### Using Foxglove

[Foxglove](https://foxglove.dev/) is used to visualize ROS messages on a local machine.

Add `data_stream` as an `ACTIVE_PROFILE` and declare `FOXGLOVE_BRIDGE_PORT=[port]` in `dev_config.local.sh`. This will launch the `foxglove.Dockerfile` container with an open port when `watod2 up` is ran. 

It exposes the port specified by the `FOXGLOVE_BRIDGE_PORT` variable, which you will need to forward to your local machine. This can either be done in the `ports` section of VS Code or by running the command `ssh -L 8765:localhost:8765 <username>@<machine>-ubuntu1.watocluster.local` on your local machine.

Then, open foxglove and add a connection `localhost:8765`, and it should connect.

### Using Carla Jupyter Notebook

Jupyter Notebook can be used to write python notebooks on a local machine that can be executed in a running container.
This container is primarily to be used to interact with Carla without having to write ROS nodes that require a lot of boilerplate

Add `carla` as an `ACTIVE_PROFILE` and declare `CARLA_NOTEBOOKS_PORT=[port]` in `dev_config.local.sh`. This will launch the `carla_notebooks.Dockerfile` container with an open port when `watod2 up` is ran. 

It exposes the port specified by the `CARLA_NOTEBOOKS_PORT` variable, which you will need to forward to your local machine. This can either be done in the `ports` section of VS Code or by running the command `ssh -L 8888:localhost:8888 <username>@<machine>-ubuntu1.watocluster.local` on your local machine.

Then, open any browser and type `localhost:8888` and press enter. In the Jupyter Notebook UI select the `carla_notebook.ipynb` file and you should be able to start coding. 

## Monorepo Info
[docs/monorepo.md](docs/monorepo.md)

### Technical Specification 
Docker, Continuous Integration: [tech_spec.md](docs/tech_spec.md)

### FAQ
[docs/faq.md](docs/faq.md)