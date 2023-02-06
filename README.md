# WATonomous Monorepo v2

Dockerized ROS2 setup for the WATonomous Autonomous Driving Software Pipeline

- [WATonomous Monorepo](#watonomous-monorepo)
  - [Getting Started](#getting-started)
  - [Description of Files](#description-of-files)
  - [Profiles](#profiles)
  - [Common Operations](#common-operations)
    - [watod2](#watod2)
    - [Remote VScode](#remote-vscode)
  - [Monorepo Info](#monorepo-info)
  - [Technical Specification](#technical-specification)
  - [Testing Infrastructure](#testing)
  - [FAQ](#faq)
    - [Every time I make a new branch I need to rebuild all the images from scratch](#every-time-i-make-a-new-branch-i-need-to-rebuild-all-the-images-from-scratch)
    - ["Invalid reference format" when using watod2](#invalid-reference-format-when-using-watod2)

## Getting Started
Read the following:
1. [docs/setup.md](docs/setup.md) How to setup our repo. 

**TLDR:** Clone the monorepo, specify active profiles, `watod2 up`. Everything is containerized, so there's little need to setup any dependencies on your end :).

2. [docs/monorepo.md](docs/monorepo.md) What is a monorepo? Why a monorepo?
3. [docs/how_to_dev.md](docs/monorepo.md) How to develop in the monorepo.

## Description of Important Files and Directories

Below is a tree diagram of the Monorepo.

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
  - This is the main bash script that you will use to interface with your containers. [More info](#watod2).
- `dev_config.sh`. 
  - dev_config.sh will create a [.env file](https://docs.docker.com/compose/env-file/) specifying environment variables for docker-compose. `watod2` automatically runs this script before running any commands. To override variables in `dev_config.sh`, create a `dev_config.local.sh` file and populate it with variables, for example `ACTIVE_PROFILES="perception path_planning"`. `dev_config.sh` will then take this file into account when building the `.env` file.
- `scripts/watod2-completion.bash`.
  - Bash autocomplete for watod2. Adapted from docker-compose. Add `source <MONO_DIR>/scripts/watod2-completion.bash` to your bashrc to use autocomplete.
- `profiles/`: 
  - This folder contains all docker-compose files specifying the services we will run. They are divided into profiles. The root-level `docker-compose.yaml` file is loaded first, then profiles are loaded after. Note that by default no profiles are enabled. To select additional profiles, overwrite `ACTIVE_PROFILES=x` in `dev_config.local.sh`. See the [docker-compose wiki](https://docs.docker.com/compose/extends/).
- `docker/`: 
  - This folder contains the `Dockerfiles` for each of our images. [Docker wiki](https://docs.docker.com/engine/reference/builder/).
- `src/`: 
  - Here is where our actual code goes. The folders in `src` will be mounted to the docker images, so changes in the `src` directory will be reflected in the containers. 

## Common Operations

### watod2

`watod2` is a wrapper for docker-compose. The format is `watod2 [watod2 options] [docker-compose options]`. See watod2 options using `watod2 -h`. `docker-compose` interface can be found here: https://docs.docker.com/compose/ 

By default, `watod2` will use and create images tagged based on your current branch. For example, `perception/debug-develop`. If you switch to a new branch, you will need to rebuild your images (automatic with `watod2 up`)

For any environment variable found in `dev-config.sh`, you can overwrite it on the command line as follows: `ENV=x ENV2=y ./watod2 ...`. For example, if I am on a different branch but I want to start `develop` images, I can use `TAG=develop ./watod2 up`

**Don't like using `./watod2`? Want to use `watod2` instead??**

We recommend you add `~/wato_monorepo_v2/` to your `$PATH` variable so that you can run `watod2` from anywhere on your computer, not just in your `~/wato_monorepo_v2` directory and add `watod2-completion.bash` to your bashrc to enable tab autocomplete:
```bash
echo "export PATH=\$PATH:\$HOME/wato_monorepo_v2/" >> ~/.bashrc
echo "source ~/wato_monorepo_v2/scripts/watod2-completion.bash" >> ~/.bashrc
source ~/.bashrc
```

**Starting Containers**: `watod2 up`
- Runs `docker-compose up` after generating your `.env` file. Your terminal will start, print out a bunch of logs, then hang while waiting for more logs. **This command does not exit**. To stop your containers, press `ctrl-c`.
- use `watod2 up -h` to see other arguments you can pass to `watod2 up`

**Stopping containers**: `watod2 down`

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

## Monorepo Info
[docs/monorepo.md](docs/monorepo.md)

## Technical Specification 
Docker, Continuous Integration: [tech_spec.md](docs/tech_spec.md)

## Testing
[docs/testing.md](docs/testing.md)

## FAQ 

### Every time I make a new branch I need to rebuild all the images from scratch

Whenever you make a new branch, new images need to be compiled. However, Docker will use the `develop` images as a cache. You probably don't have the `develop` images downloaded on your machine.

Try these steps:
```bash
# Log into the docker registry
$ docker login git.uwaterloo.ca:5050
# Pull the latest develop images
$ TAG=develop ./watod2 --all pull
# Start your containers
$ ./watod2 up
```

### "Invalid reference format" when using watod2

If you get an error such as: `ERROR: no such image: git.uwaterloo.ca:5050/watonomous/wato_monorepo/perception:debug-charles/test: invalid reference format`

This is because your branch name has a slash (`/`) in it. This breaks the image naming system that docker uses. Please change your branch name to use dashes `-` instead.

```bash
git checkout -b <NEW_BRANCH_NAME>
# VERIFY all of your changes are saved in your new branch
git branch -d <OLD_BRANCH_NAME>
```