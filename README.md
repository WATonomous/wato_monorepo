# WATonomous Monorepo v2

Dockerized ROS2 setup for the WATonomous Autonomous Driving Software Pipeline

- [WATonomous Monorepo](#watonomous-monorepo)
  - [Getting Started](#getting-started)
  - [Description of Files](#description-of-files)
  - [Profiles](#profiles)
  - [Common Operations](#common-operations)
    - [watod2](#watod2)
    - [Remote VScode](#remote-vscode)
    - [VNC](#vnc)
    - [CARLA](#carla)
    - [Playing Rosbags](#playing-rosbags)
  - [Production Workflow](#production-workflow)
  - [Monorepo Info](#monorepo-info)
  - [Technical Specification](#technical-specification)
  - [Testing Infrastructure](#testing)
  - [Debugging](#debugging)
  - [FAQ](#faq)
    - [Every time I make a new branch I need to rebuild all the images from scratch](#every-time-i-make-a-new-branch-i-need-to-rebuild-all-the-images-from-scratch)
    - ["Invalid reference format" when using watod2](#invalid-reference-format-when-using-watod2)

## Getting Started
**READ THIS**: [docs/setup.md](docs/setup.md)

## Description of Files

- `watod2`. 
  - This is the main bash script that you will use to interface with your containers. [More info](#watod2).
- `dev_config.sh`. 
  - dev_config.sh will create a [.env file](https://docs.docker.com/compose/env-file/) specifying environment variables for docker-compose. `watod2` automatically runs this script before running any commands. To override variables in `dev_config.sh`, create a `dev_config.local.sh` file and populate it with variables, for example `ACTIVE_PROFILES="perception path_planning"`. `dev_config.sh` will then take this file into account when building the `.env` file.
- `scripts/format_files.sh`.
  - CLANG formatting script. Run this script to format your c++ files before merging any code into develop. Make sure to commit any changes in progress before formatting your files.
- `scripts/watod2-completion.bash`.
  - Bash autocomplete for watod2. Adapted from docker-compose. Add `source <MONO_DIR>/scripts/watod2-completion.bash` to your bashrc to use autocomplete.
- `profiles/`: 
  - This folder contains all docker-compose files specifying the services we will run. They are divided into profiles. The root-level `docker-compose.yaml` file is loaded first, then profiles are loaded after. Note that by default no profiles are enabled. To select additional profiles, overwrite `ACTIVE_PROFILES=x` in `dev_config.local.sh`. See the [docker-compose wiki](https://docs.docker.com/compose/extends/).
- `docker/`: 
  - This folder contains the `Dockerfiles` for each of our images. [Docker wiki](https://docs.docker.com/engine/reference/builder/).
- `src/`: 
  - Here is where our actual code goes. The folders in `src` will be mounted to the docker images, so changes in the `src` directory will be reflected in the containers. 
- `code_server_config/`: 
  - This is where you will store your VSCode development setup. This is personalized, and you should not commit any personal changes to this folder.

## Profiles

You need to specify what profiles you want to use by setting `ACTIVE_PROFILES` in your `dev_config.local.sh`. For example, your `dev_config.local.sh` might include `ACTIVE_PROFILES="carla tools matlab perception path_planning"`. 

Profiles are defined as `profiles/docker-compose.<PROFILE>.yaml`. If you want to overwrite your current profile and run the command against all profiles, use `watod2 --all COMMAND`

```bash
from dev_config.sh
# Space-delimited list of active profiles to run, defined in docker-compose.yaml.
# Possible values:
#   - carla         :   if enabled, starts the carla simulation tools
#   - tools         :   starts any debug and visualization tools
#   - matlab        :   starts the matlab container
#   - perception    :   starts the all perception modules
#   - path_planning :   starts the all path planning modules
# Example override in dev_config.local.sh: 
#   ACTIVE_PROFILES="carla tools matlab perception path_planning"

ACTIVE_PROFILES=${ACTIVE_PROFILES:-""}
```

## Common Operations

### watod2

`watod2` is a wrapper for docker-compose. The format is `watod2 [watod2 options] [docker-compose options]`. See watod2 options using `watod2 -h`. `docker-compose` interface can be found here: https://docs.docker.com/compose/ 

By default, `watod2` will use and create images tagged based on your current branch. For example, `perception/debug-develop`. If you switch to a new branch, you will need to rebuild your images (automatic with `watod2 up`)

For any environment variable found in `dev-config.sh`, you can overwrite it on the command line as follows: `ENV=x ENV2=y ./watod2 ...`. For example, if I am on a different branch but I want to start `develop` images, I can use `TAG=develop ./watod2 up`

**Starting Containers**: `watod2 up`
- Runs `docker-compose up` after generating your `.env` file. Your terminal will start, print out a bunch of logs, then hang while waiting for more logs. **This command does not exit**. To stop your containers, press `ctrl-c`.
- use `watod2 up -h` to see other arguments you can pass to `watod2 up`

**Stopping containers**: `watod2 down`

**Building images**: `watod2 build`

**Seeing exposed ports**: `watod2 --ports`
- Your docker containers expose a certain number of applications that can be accessed publicly. For example, [VNC](https://en.wikipedia.org/wiki/Virtual_Network_Computing) or [code-server](https://github.com/cdr/code-server)
- Start your containers with `watod2 up` then in another terminal use `watod2 --ports`
- `watod2 -lp` will  also print information if you want to forward the ports from the external server to your local machine over SSH.
- `watod2 -lc [local_base_port]` will generate a [~/.ssh/config file](https://linuxize.com/post/using-the-ssh-config-file/) for you to copy to your local machine. A newly generated config file will always be up to date with VM hostnames, watod2 ports forwards, etc... and in our oppinion is the best way to configure your communication with WATO's server cluster. The optional `[local_base_port]` argument allows you to define your own port range on your local machine. For example, if the `pp_env_model` service is running on remote port `3864`, with your remote base port being `3860`, then running `watod2 -lc 1000` will place a forwarding rule in the config to forward the  `pp_env_model`'s code server to `localhost:1004`.
  
**Opening a shell inside a docker container**: `watod2 -t <SERVICE_NAME>`
- Opens a bash shell into the specified service. Find a list of your services using `watod2 ps --services`
- From here, you can execute commands inside the docker container. For example, ROS commands. 

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

### VNC

Open a VNC client locally and connect to the vnc server hosted by the `gui_tools` service. Find the server address using `$ ./watod2 -p`. For example, mine is `wato-tr.uwaterloo.ca:00180`. A popular VNC client software is [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/).

You should see a desktop similar to this:

![image](https://user-images.githubusercontent.com/5977478/81488679-d5b15980-9220-11ea-8d1e-1aa91ec2fc82.png)

You can now run ROS commands with an interactive GUI. For example, open a shell into the `gui_tools` container: `$ watod2 -t gui_tools` and start RVIZ: `$ rosrun rviz rviz`. See [carla_config](src/simulation/carla_config/README.md) for an in-depth tutorial on how to use CARLA simulation.

Note that you need to execute any gui-related commands **inside** the `gui_tools` container. Otherwise, you'll get a `no display found` error.

### CARLA

Please see the CARLA folder at [`src/simulation/carla_config`](src/simulation/carla_config/README.md) . To use CARLA simulation, add the `carla` profile to your `dev-config.local.sh`.

W21 CARLA Training Video: https://drive.google.com/file/d/1k4XJ3ps6fqM-2Ntl7fas__O_y5NxS2ph/view

To drive the car around manually in CARLA, use `roslaunch carla_manual_control carla_manual_control.launch` in your `gui_tools` container and connect to the VNC. a PyGame window will show up in the VNC window. From there, press `H` to see instructions.

### Playing Rosbags

> :warning: **The following setup is being testing and is subject to change. If you encounter issues or have suggestions, please post them in #software-general.**

Enable `bag_bridge` as an `ACTIVE_PROFILE` in `dev_config.local.sh`. Also, remove `carla` if you have that active, because the Rosbag we will be playing replaces the need for simulated inputs to the autonomous pipeline.

Run `watod2 up -d bag_bridge` (or however you want to launch the `bag_bridge` service). 

The working directory of the `bag_bridge` container should be (at time of writing) equivalent to the `/mnt/rawdata1/bags/year4/april_17_test` directory on the Rugged server (in the future you will be able to able to customize what directory you want to mount). To confirm this, run `watod2 run bag_bridge ls` to view the available bags. 

Now, using `watod2 run bag_bridge [ROSBAG COMMAND]` you can run any `rosbag ...` command as documented here: http://wiki.ros.org/rosbag/Commandline. You probably want to explore `rosbag play ...`: http://wiki.ros.org/rosbag/Commandline#rosbag_play. For example, `watod2 run bag_bridge rosbag play -l --clock test8_left_turn_symbol.bag` will play the specified bag on a loop (`-l`) and the ROS clock topic (`/clock`) will be published according to the message timestamps in the bag. 

After playing a bag, launch the autuonomous pipeline as usual, and start debugging whatever bug you're interested in. If you run into issues with reproducing the autonomous pipeline behavior using the information in the bag (e.g. issues with the transform tree, HD map not loading, etc...) post in #software-general and you will be assisted. 


## Production Workflow

Instructions pertain to running autonomously on the Rugged. 

1. Download & set up the monorepo as normal.
2. Set your `dev_config.local.sh` as follows:
```bash
ACTIVE_PROFILES="production"
TARGET_STAGE="repo"
```
3. Run `watod2 up` to start the software stack and sensor drivers. Verify that all sensors are nominal.
4. Run `ros2 topic echo /feedback_desired_output` to inspect the torque and steering values output by Path Planning. Torque is in `NM` and should be a value between `100-600`. Steering is in radians and should be approximately zero.
5. Once you are confident all systems are running correctly, put the Bolty in drive and disengage the brake pedal. The safety driver should be ready to perform a manual override at any time.
6. Run `watod2 launch_autonomous` in a separate terminal. Once you press enter, the CAN interface will launch and the vehicle will be in autonomous mode.
7. Press ctrl-c twice to disengate autonomous mode.

## Monorepo Info
[docs/monorepo.md](docs/monorepo.md)

## Technical Specification 
Docker, Continuous Integration: [tech_spec.md](docs/tech_spec.md)

## Testing
[docs/testing.md](docs/testing.md)

## Debugging
[docs/debugging.md](docs/debugging.md)

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