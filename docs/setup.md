# Getting Started

## System Requirements
- Docker-compose 1.29.2 or later
- Docker 20.10.18 or later
- Nvidia GPU (for simulation)

STRONGLY Recommended: Use a WATonomous server 

## Terminology
- Local Machine: Your personal machine
- Host Machine: The machine that you will be developing on. We recommend you use our WATonomous servers. The host and local machine can be the same.

## Prerequisites
These are just some general prereqs to setup our monorepo. It's better that you follow our Server Onboarding first and enter our server. After that, no need to worry about these prereqs. You can clone the monorepo and get going!

1. **UW VPN:** You are either on-campus connected to Eduroam or you have the [UW VPN](https://uwaterloo.ca/information-systems-technology/services/virtual-private-network-vpn) installed on your local machine. **You only need the VPN to use GUI tools (e.g. Carla).**
2. **Github SSH keys:** You have a [github ssh key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).
    * If you have a github ssh key on your local machine, you can [forward it to the server](https://docs.github.com/en/developers/overview/using-ssh-agent-forwarding).
3. **Authorized WATO user:** You have registered a user for yourself on WATO's servers as per the instrucitons in [`ansible-config`](https://git.uwaterloo.ca/WATonomous/ansible-config). Note that you can use your local machine's SSH public key for a passwordless login during the profile creation in [`ansible-config`](https://git.uwaterloo.ca/WATonomous/ansible-config). You can always update your profile should your password or SSH key(s) change. Members should consult with their manager, tech lead, or an infrastructure member about which server to use. Log in to a server VM with `$ ssh yourusername@vm-domain-name` where `vm-domain-name` can be found in [the server details spreadsheet](https://docs.google.com/spreadsheets/d/141TjJNwrWngtkDIp-4q6c1888kq4EBy0wa3FsS29BnE) (remember to use your watonomous.ca credentials).
4. **ROS Knowledge:** You know how ROS works and how an ament workspace is structured. Otherwise this guide may be confusing. See the [ROS2 Tutorials](http://docs.ros.org.ros.informatik.uni-freiburg.de/en/foxy/Tutorials.html) for more information.
5. **Docker:** You have access to Docker on the host machine. Verify this by running `$ docker info`. You shouldn't see any errors.
6. **Nvidia GPU:** The host machine has an Nvidia GPU. Otherwise you won't be able to use our Carla simulator. Verify this with `nvidia-smi`.

Breifly familarize yourself with Docker and Docker Compose before continuing.

## Setting up your workspace

1. Clone this repo onto the host machine on using `$ git clone git@github.com:WATonomous/wato_monorepo_v2.git`. We recommend you clone the repo into your home directory, `~`
2. Ensure you are on the develop branch: `$ cd ~/wato_monorepo_v2 && git checkout develop && git pull`
3. Create a file `dev_config.local.sh` in `~/wato_monorepo_v2` and add the line 
```
#!/bin/bash
from dev_config.sh

ACTIVE_PROFILES="${specific profiles}"
```

You need to specify what profiles you want to use by setting `ACTIVE_PROFILES` in your `dev_config.local.sh`. For example, your `dev_config.local.sh` might include `ACTIVE_PROFILES="carla tools matlab perception path_planning"`. 

Profiles are defined as `profiles/docker-compose.<PROFILE>.yaml`. If you want to overwrite your current profile and run the command against all profiles, use `watod2 --all COMMAND`

```bash
from dev_config.sh
# Space-delimited list of active profiles to run, defined in docker-compose.yaml.
# Possible values:
#   - production    		:   configs for all containers required in production
#   - samples           :   starts sample ROS2 pubsub nodes
# Example override in dev_config.local.sh: 
#   ACTIVE_PROFILES="samples production"

ACTIVE_PROFILES=${ACTIVE_PROFILES:-""}
```

4. Run `$ ./watod2 pull` to pull latest docker images from our container registry.
5. Run `$ ./watod2 up` to run the Docker images, and watch for any errors. If succesful, this will create a variety of containers prefixed with `${USER}_`. This command does not exit, and will continue to monitor the logs from your containers.
6. In another terminal, enter `$ ./watod2 ps` to see a list of your open containers.
7. When you're done, press `ctrl-c` in your original terminal to close the containers. 

The `watod2` will be your the main way that you interact with your containers. `watod2` is a wrapper for `docker-compose` that automates the setup of environment varaibles and has some additional functionality. Get more information about `watod2` using: `$ watod2 -h`.

## Don't like using `./watod2`? Want to use `watod2` instead??
We recommend you add `~/wato_monorepo_v2/` to your `$PATH` variable so that you can run `watod2` from anywhere on your computer, not just in your `~/wato_monorepo_v2` directory and add `watod2-completion.bash` to your bashrc to enable tab autocomplete:
```bash
echo "export PATH=\$PATH:\$HOME/wato_monorepo_v2/" >> ~/.bashrc
echo "source ~/wato_monorepo_v2/scripts/watod2-completion.bash" >> ~/.bashrc
source ~/.bashrc
```
## Eager to begin development?
[docs/how_to_dev.md](docs/how_to_dev.md)