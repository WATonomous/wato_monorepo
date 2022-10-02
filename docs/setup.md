# Getting Started

## System Requirements
- Docker-compose 1.28.0-rc1 or later
- Docker 19.03.12 or later
- Nvidia GPU (for simulation)

Recommended: Use a WATonomous server 

## Terminology
- Local Machine: Your personal machine
- Host Machine: The machine that you will be developing on. We recommend you use our WATonomous servers. The host and local machine can be the same.

## Prerequisites

1. **UW VPN:** You are either on-campus connected to Eduroam or you have the [UW VPN](https://uwaterloo.ca/information-systems-technology/services/virtual-private-network-vpn) installed on your local machine. You need the VPN to access our servers.
2. **GitLab SSH keys:** You have a [gitlab ssh key](https://docs.gitlab.com/ee/ssh/) with git.uwaterloo.ca on the server you will be using. You can check this running `$ ssh -T ist-git@git.uwaterloo.ca` on the server, you should NOT be required to enter a password and should see your UWaterloo username in the output.
    * If you have a gitlab ssh key on your local machine, you can [forward it to the server](https://docs.github.com/en/developers/overview/using-ssh-agent-forwarding).
3. **Authorized WATO user:** You have registered a user for yourself on WATO's servers as per the instrucitons in [`ansible-config`](https://git.uwaterloo.ca/WATonomous/ansible-config). Note that you can use your local machine's SSH public key for a passwordless login during the profile creation in [`ansible-config`](https://git.uwaterloo.ca/WATonomous/ansible-config). You can always update your profile should your password or SSH key(s) change. Members should consult with their manager, tech lead, or an infrastructure member about which server to use. Log in to a server VM with `$ ssh yourusername@vm-domain-name` where `vm-domain-name` can be found in [the server details spreadsheet](https://docs.google.com/spreadsheets/d/141TjJNwrWngtkDIp-4q6c1888kq4EBy0wa3FsS29BnE) (remember to use your watonomous.ca credentials).
4. **ROS Knowledge:** You know how ROS works and how a catkin workspace is structured. Otherwise this guide may be confusing. See the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) for more information.
5. **Docker:** You have access to Docker on the host machine. Verify this by running `$ docker info`. You shouldn't see any errors.
6. **Nvidia GPU:** The host machine has an Nvidia GPU. Otherwise you won't be able to use our Carla simulator. Verify this with `nvidia-smi`.

Breifly familarize yourself with Docker and Docker Compose before continuing.

## Setting up your workspace

1. Clone this repo onto the host machine on using `$ git clone ist-git@git.uwaterloo.ca:WATonomous/wato_monorepo.git`. We recommend you clone the repo into your home directory, `~/wato_monorepo`
2. Ensure you are on the develop branch: `$ cd ~/wato_monorepo && git checkout develop && git pull`
3. Log into UW Gitlab's Docker Registry to get access to WATonomous' Docker Images: `$ docker login git.uwaterloo.ca:5050`.
4. Create a file `dev_config.local.sh` in `~/wato_monorepo` and add the line `ACTIVE_PROFILES="carla path_planning tools"`. See [the main `Readme.md`](https://git.uwaterloo.ca/WATonomous/wato_monorepo/-/blob/develop/README.md#profiles) for more info on active profiles.
5. Run `$ ./watod pull` to pull latest docker images from our container registry.
6. Run `$ ./watod up` to run the Docker images, and watch for any errors. If succesful, this will create a variety of containers prefixed with `${USER}_`. This command does not exit, and will continue to monitor the logs from your containers.
7. In another terminal, enter `$ ./watod ps` to see a list of your open containers.
8. When you're done, press `ctrl-c` in your original terminal to close the containers. 

The `watod` will be your the main way that you interact with your containers. `watod` is a wrapper for `docker-compose` that automates the setup of environment varaibles and has some additional functionality. Get more information about `watod` using: `$ watod -h`.

We recommend you add `~/wato_monorepo/` to your `$PATH` variable so that you can run `watod` from anywhere on your computer, not just in your `~/wato_monorepo` directory:
```bash
$ echo "export PATH=\$PATH:\$HOME/wato_monorepo/" >> ~/.bashrc
$ source ~/.bashrc
```
