# WATonomous Monorepo

Dockerized ROS setup for the the WATonomous Autonomous Driving Software Pipeline

Maintainers: Charles Zhang, Rowan Dempster

- [WATonomous Monorepo](#watonomous-monorepo)
  - [Getting Started](#getting-started)
  - [Description of Files](#description-of-files)
  - [Profiles](#profiles)
  - [Common Operations](#common-operations)
    - [watod](#watod)
    - [code-server](#code-server)
    - [Remote VScode](#remote-vscode)
    - [VNC](#vnc)
    - [CARLA](#carla)
    - [Matlab](#matlab)
    - [Playing Rosbags](#playing-rosbags)
  - [Production Workflow](#production-workflow)
  - [Monorepo Info](#monorepo-info)
  - [Technical Specification](#technical-specification)
  - [FAQ](#faq)
    - [Every time I make a new branch I need to rebuild all the images from scratch](#every-time-i-make-a-new-branch-i-need-to-rebuild-all-the-images-from-scratch)
    - [Only the rosmaster service starts when I run watod up](#only-the-rosmaster-service-starts-when-i-run-watod-up)
    - ["Invalid reference format" when using watod](#invalid-reference-format-when-using-watod)
    - [How do I build an image outside of docker-compose?](#how-do-i-build-an-image-outside-of-docker-compose)
    - [I'm on a server but it doesn't have ROS installed. Is there an easy way to use ros images on my workspace without using watod?](#im-on-a-server-but-it-doesnt-have-ros-installed-is-there-an-easy-way-to-use-ros-images-on-my-workspace-without-using-watod)

## Getting Started
**READ THIS**: [docs/setup.md](docs/setup.md)

### bashrc

Add `watod` to your PATH to use `watod` from anywhere. 
Add `scripts/watod-completions.sh` to your bashrc to use tab autocomplete with `watod`.
```bash
echo "export PATH=\$PATH:\$HOME/wato_monorepo/" >> ~/.bashrc
echo "source ~/wato_monorepo/scripts/watod-completion.bash" >> ~/.bashrc
source ~/.bashrc
```


## Description of Files

- `watod`. 
  - This is the main bash script that you will use to interface with your containers. [More info](#watod).
- `dev_config.sh`. 
  - dev_config.sh will create a [.env file](https://docs.docker.com/compose/env-file/) specifying environment variables for docker-compose. `watod` automatically runs this script before running any commands. To override variables in `dev_config.sh`, create a `dev_config.local.sh` file and populate it with variables, for example `ACTIVE_PROFILES="perception path_planning"`. `dev_config.sh` will then take this file into account when building the `.env` file.
- `scripts/format_files.sh`.
  - CLANG formatting script. Run this script to format your c++ files before merging any code into develop. Make sure to commit any changes in progress before formatting your files.
- `scripts/watod-completion.bash`.
  - Bash autocomplete for watod. Adapted from docker-compose. Add `source <MONO_DIR>/scripts/watod-completion.bash` to your bashrc to use autocomplete.
- `docker-compose.yaml`
  - Base docker-compose file launching a personalized rosmaster. See [docker-compose spec](https://github.com/compose-spec/compose-spec/blob/master/spec.md).
- `profiles/`: 
  - This folder contains all docker-compose files specifying the services we will run. They are divided into profiles. The root-level `docker-compose.yaml` file is loaded first, then profiles are loaded after. Note that by default no profiles are enabled. To select additional profiles, overwrite `ACTIVE_PROFILES=x` in `dev_config.local.sh`. See the [docker-compose wiki](https://docs.docker.com/compose/extends/).
- `docker/`: 
  - This folder contains the `Dockerfiles` for each of our images. [Docker wiki](https://docs.docker.com/engine/reference/builder/).
- `src/`: 
  - Here is where our actual code goes. The folders in `src` will be mounted to the docker images, so changes in the `src` directory will be reflected in the containers. 
- `code_server_config/`: 
  - This is where you will store your VSCode development setup. This is personalized, and you should not commit any personal changes to this folder.
- `.matlab_licenses/`: 
  - Full Autodrive MATLAB license for use in docker containers. Note will only work in the `matlab` docker image.
- `.gitlab/ci/`:
  - Folder for Gitlab CI scripts
- `.gitlab_ci.yml`:
  - Yaml specification for CI. [GitLab Wiki](https://docs.gitlab.com/ee/ci/yaml/gitlab_ci_yaml.html).

## Profiles

**By default, `watod` commands will will only affect the rosmaster node**.

You need to specify what profiles you want to use by setting `ACTIVE_PROFILES` in your `dev_config.local.sh`. For example, your `dev_config.local.sh` might include `ACTIVE_PROFILES="carla tools matlab perception path_planning"`. 

Profiles are defined as `profiles/docker-compose.<PROFILE>.yaml`. If you want to overwrite your current profile and run the command against all profiles, use `watod --all COMMAND`

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

### watod

`watod` is a wrapper for docker-compose. The format is `watod [watod options] [docker-compose options]`. See watod options using `watod -h`. `docker-compose` interface can be found here: https://docs.docker.com/compose/ 

By default, `watod` will use and create images tagged based on your current branch. For example, `perception/debug-develop`. If you switch to a new branch, you will need to rebuild your images (automatic with `watod up`)

For any environment variable found in `dev-config.sh`, you can overwrite it on the command line as follows: `ENV=x ENV2=y ./watod ...`. For example, if I am on a different branch but I want to start `develop` images, I can use `TAG=develop ./watod up`

**Starting Containers**: `watod up`
- Runs `docker-compose up` after generating your `.env` file. Your terminal will start, print out a bunch of logs, then hang while waiting for more logs. **This command does not exit**. To stop your containers, press `ctrl-c`.
- use `watod up -h` to see other arguments you can pass to `watod up`

**Stopping containers**: `watod down`

**Building images**: `watod build`

**Seeing exposed ports**: `watod --ports`
- Your docker containers expose a certain number of applications that can be accessed publicly. For example, [VNC](https://en.wikipedia.org/wiki/Virtual_Network_Computing) or [code-server](https://github.com/cdr/code-server)
- Start your containers with `watod up` then in another terminal use `watod --ports`
- `watod -lp` will  also print information if you want to forward the ports from the external server to your local machine over SSH.
- `watod -lc [local_base_port]` will generate a [~/.ssh/config file](https://linuxize.com/post/using-the-ssh-config-file/) for you to copy to your local machine. A newly generated config file will always be up to date with VM hostnames, watod ports forwards, etc... and in our oppinion is the best way to configure your communication with WATO's server cluster. The optional `[local_base_port]` argument allows you to define your own port range on your local machine. For example, if the `pp_env_model` service is running on remote port `3864`, with your remote base port being `3860`, then running `watod -lc 1000` will place a forwarding rule in the config to forward the  `pp_env_model`'s code server to `localhost:1004`.
  
**Opening a shell inside a docker container**: `watod -t <SERVICE_NAME>`
- Opens a bash shell into the specified service. Find a list of your services using `watod ps --services`
- From here, you can execute commands inside the docker container. For example, ROS commands. 

### code-server

1. Run `$ ./watod up` to start your containers. Wait for your containers to start.

In a new terminal, run `$ ./watod --ports` or `$ ./watod -p`. you should see something similar to this. The ports for your host machine and user will be different
Open a Code-server URL in your local machine's browser.  eg: `http://wato-tr.uwaterloo.ca:00185` is my perception container. Enter the password. The default password is `pass` and is configurable in `dev-config.sh`.

You should see a vscode-like editor:

![image](https://user-images.githubusercontent.com/5977478/81488744-aa7b3a00-9221-11ea-929d-30a5398140f0.png) 

You can now edit code in the various containers. Each docker container will have one code server. **Only certain folders are shared between the docker container and the host**. To avoid losing work, only modify files in the `code-server`. Create new files on the host (outside the docker container).

If you get an error: `catkin: command not found` or `<ROS_PACKAGE>: command not found`, run `$ source ~/catkin_ws/devel/setup.bash` then retry.

If you want a more native feel (e.g. CNTR+TAB to open a new IDE tab instead of Chrome tab), you can easily forward the host machine ports to your local machine ports using `$ ./watod --local-ports` or `$ ./watod -lp`. You should see something like:
```
Ports exposed by running containers:
carla_ros_bridge exposes a Code Server at http://wato-tr.uwaterloo.ca:10305
	To forward it locally, run
		ssh -N -L 10305:localhost:10305 rowan@wato-tr.uwaterloo.ca
	on your local machine and go to http://localhost:10305

gui_tools exposes a VNC Server at wato-tr.uwaterloo.ca:10300
...
To forward all ports locally:
ssh -N -L 10305:localhost:10305 -L 10301:localhost:10301 -L 10303:localhost:10303 -L 10304:localhost:10304 -L 10302:localhost:10302 rowan@wato-tr.uwaterloo.ca
```

Copy the last line of the output (`ssh -N -L 10305:localhost:10305 -L 10301:localhost:10301 ... rowan@wato-tr.uwaterloo.ca`) and execute it on your local machine. Note that you will need [SSH](https://docs.microsoft.com/en-us/windows-server/administration/openssh/openssh_install_firstuse) installed locally. The SSH command forwards all the ports found by `$ ./watod --ports` to the your `localhost` (ie local machine). So you can now navigate to, for example, `http://localhost:10302` instead of `http://wato-tr.uwaterloo.ca:10302`. The SSH forwarding will stay active until you ^C it, or the bash session is closed.

What local forwarding allows you to do is install a [Chrome Progressive Web App (PWA)](https://support.google.com/chrome/answer/9658361?co=GENIE.Platform%3DDesktop&hl=en) for Code Server onto your local machine. Do so by navigating to the **localhost** Code Server in Chrome and following the instructions in the [PWA link](https://support.google.com/chrome/answer/9658361?co=GENIE.Platform%3DDesktop&hl=en). Now you can re-name the installed PWA (so you know what container it belongs to), keep it on your local machine's desktop/toolbar, and pretty much do anything you can do with a native app!

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

Open a VNC client locally and connect to the vnc server hosted by the `gui_tools` service. Find the server address using `$ ./watod -p`. For example, mine is `wato-tr.uwaterloo.ca:00180`. A popular VNC client software is [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/).

You should see a desktop similar to this:

![image](https://user-images.githubusercontent.com/5977478/81488679-d5b15980-9220-11ea-8d1e-1aa91ec2fc82.png)

You can now run ROS commands with an interactive GUI. For example, open a shell into the `gui_tools` container: `$ watod -t gui_tools` and start RVIZ: `$ rosrun rviz rviz`. See [carla_config](src/simulation/carla_config/README.md) for an in-depth tutorial on how to use CARLA simulation.

Note that you need to execute any gui-related commands **inside** the `gui_tools` container. Otherwise, you'll get a `no display found` error.

### CARLA

Please see the CARLA folder at [`src/simulation/carla_config`](src/simulation/carla_config/README.md) . To use CARLA simulation, add the `carla` profile to your `dev-config.local.sh`.

W21 CARLA Training Video: https://drive.google.com/file/d/1k4XJ3ps6fqM-2Ntl7fas__O_y5NxS2ph/view

To drive the car around manually in CARLA, use `roslaunch carla_manual_control carla_manual_control.launch` in your `gui_tools` container and connect to the VNC. a PyGame window will show up in the VNC window. From there, press `H` to see instructions.

### Matlab

Enable `matlab` as an `ACTIVE_PROFILE` in `dev_config.local.sh`

Open a VNC client locally and connect the vnc server hosted by the `matlab` service. Find the server address using `$ ./watod -p`. The container has access to the monorepo source files in `/home/docker/src`. To run matlab, open a terminal and enter `matlab`. MATLAB has all r2020b toolboxes installed. You will also be able to connect to the ROS network from MATLAB by running `rosinit()`. 

More information: https://git.uwaterloo.ca/watorace/matlab-r2020b

### Playing Rosbags

> :warning: **The following setup is being testing and is subject to change. If you encounter issues or have suggestions, please post them in #software-general.**

Enable `bag_bridge` as an `ACTIVE_PROFILE` in `dev_config.local.sh`. Also, remove `carla` if you have that active, because the Rosbag we will be playing replaces the need for simulated inputs to the autonomous pipeline.

Run `watod up -d bag_bridge` (or however you want to launch the `bag_bridge` service). 

The working directory of the `bag_bridge` container should be (at time of writing) equivalent to the `/mnt/rawdata1/bags/year4/april_17_test` directory on the Rugged server (in the future you will be able to able to customize what directory you want to mount). To confirm this, run `watod run bag_bridge ls` to view the available bags. 

Now, using `watod run bag_bridge [ROSBAG COMMAND]` you can run any `rosbag ...` command as documented here: http://wiki.ros.org/rosbag/Commandline. You probably want to explore `rosbag play ...`: http://wiki.ros.org/rosbag/Commandline#rosbag_play. For example, `watod run bag_bridge rosbag play -l --clock test8_left_turn_symbol.bag` will play the specified bag on a loop (`-l`) and the ROS clock topic (`/clock`) will be published according to the message timestamps in the bag. 

After playing a bag, launch the autuonomous pipeline as usual, and start debugging whatever bug you're interested in. If you run into issues with reproducing the autonomous pipeline behavior using the information in the bag (e.g. issues with the transform tree, HD map not loading, etc...) post in #software-general and you will be assisted. 



## Production Workflow

Instructions pertain to running autonomously on the Rugged. 

1. Download & set up the monorepo as normal.
2. Set your `dev_config.local.sh` as follows:
```bash
ACTIVE_PROFILES="production"
TARGET_STAGE="repo"
```
3. Run `watod up` to start the software stack and sensor drivers. Verify that all sensors are nominal.
4. Run `rostopic echo /feedback_desired_output` to inspect the torque and steering values output by Path Planning. Torque is in `NM` and should be a value between `100-600`. Steering is in radians and should be approximately zero.
5. Once you are confident all systems are running correctly, put the Bolty in drive and disengage the brake pedal. The safety driver should be ready to perform a manual override at any time.
6. Run `watod launch_autonomous` in a separate terminal. Once you press enter, the CAN interface will launch and the vehicle will be in autonomous mode.
7. Press ctrl-c twice to disengate autonomous mode.

## Monorepo Info
[docs/monorepo.md](docs/monorepo.md)

## Technical Specification 
Docker, Continuous Integration: [tech_spec.md](docs/tech_spec.md)

## FAQ 

### Every time I make a new branch I need to rebuild all the images from scratch

Whenever you make a new branch, new images need to be compiled. However, Docker will use the `develop` images as a cache. You probably don't have the `develop` images downloaded on your machine.

Try these steps:
```bash
# Log into the docker registry
$ docker login git.uwaterloo.ca:5050
# Pull the latest develop images
$ TAG=develop ./watod --all pull
# Start your containers
$ ./watod up
```

### Only the rosmaster service starts when I run watod up

You likely haven't configured your profiles. By default, we only start the rosmaster node. See [Selecting which services to run](#selecting-which-services-to-run).

### "Invalid reference format" when using watod

If you get an error such as: `ERROR: no such image: git.uwaterloo.ca:5050/watonomous/wato_monorepo/perception:debug-charles/test: invalid reference format`

This is because your branch name has a slash (`/`) in it. This breaks the image naming system that docker uses. Please change your branch name to use dashes `-` instead.

```bash
git checkout -b <NEW_BRANCH_NAME>
# VERIFY all of your changes are saved in your new branch
git branch -d <OLD_BRANCH_NAME>
```

### How do I build an image outside of docker-compose?

Navidate to the folder with the docker image.
`$ docker build -t <NAME> .`

### I'm on a server but it doesn't have ROS installed. Is there an easy way to use ros images on my workspace without using watod?

Run the ros:noetic image mounting the monorepo into the image at `/wato_monorepo`
`cd ~/wato_monorepo && docker run -it -v $(pwd):/wato_monorepo ros:noetic /bin/bash`
In the image, navigate to `/wato_monorepo`
