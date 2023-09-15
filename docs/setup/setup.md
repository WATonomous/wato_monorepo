# Getting Started

## Terminology
- Local Machine: Your personal machine
- Host Machine: The machine that you will be developing on. We recommend you use our WATonomous servers. The host and local machine can be the same.

## System Requirements
- Docker-compose 1.29.2 or later
- Docker 20.10.18 or later
- Nvidia GPU(s)

Note: These are all available on WATonomous servers by default.

## Prerequisites

1. **UW VPN:** You only need the VPN to use GUI tools (e.g. Carla). You can either be on-campus connected to Eduroam or have the [UW VPN](https://uwaterloo.ca/information-systems-technology/services/virtual-private-network-vpn) installed on your local machine.
2. **Github SSH keys:** You have a [github ssh key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).
    * If you have a github ssh key on your local machine, you can [forward it to the server](https://docs.github.com/en/developers/overview/using-ssh-agent-forwarding).
3. **Authorized WATO user:** You have registered a user for yourself on WATO's servers through [`the server status page`](https://status.watonomous.ca/). Members should consult with their manager, tech lead, or an infrastructure member about which server to use. Log in to a server VM using the commands available on [`the server status page`](https://status.watonomous.ca/) (you can click the vm boxes, remember to use your watonomous.ca credentials).
    * For additional information on WATO's servers and detailed instructions on accessing the server, see the [meeting notes](https://docs.google.com/document/d/1AP_DjD4oRfWWy2d3EMQheVwnTuTZH0-zBFiAqYyk7Pc/edit?usp=sharing) from a previous Server Access Onboarding meeting.
4. **ROS2 Knowledge:** You know how ROS2 works and how a ROS2 workspace is structured. Otherwise this guide may be confusing. See the [ROS2 Tutorials](http://docs.ros.org.ros.informatik.uni-freiburg.de/en/foxy/Tutorials.html) for more information.
5. **Docker:** You are familiar with Docker and Docker Compose and you have access to Docker on the host machine. Verify this by running `$ docker info`. You shouldn't see any errors.
6. **Nvidia GPU:** The host machine has an Nvidia GPU. Preferably many to be able to run all the AI we have written. Verify this with `nvidia-smi`.

## Setting up your workspace

1. Clone this repo onto the host machine on using `$ git clone git@github.com:WATonomous/wato_monorepo_v2.git`. We recommend you clone the repo into your home directory, `~`
2. Create a file `watod-config.local.sh` in `~/wato_monorepo_v2` and add the line 
    ```
    #!/bin/bash
    from watod-config.sh

    ACTIVE_PROFILES="${specific profiles}"
    ```
    See [docs/dev/profiles.md](../dev/profiles.md) for more info on active profiles.

3. Run `$ ./watod pull` to pull latest docker images from our container registry.
4. Run `$ ./watod up` to run the Docker images, and watch for any errors. If succesful, this will create a variety of containers prefixed with `${USER}_`. This command does not exit, and will continue to monitor the logs from your containers.
5. In another terminal, enter `$ ./watod ps` to see a list of your open containers.
6. When you're done, press `ctrl-c` in your original terminal to close the containers. 

The `watod` will be your the main way that you interact with your containers. `watod` is a wrapper for `docker-compose` that automates the setup of environment varaibles and has some additional functionality. Get more information about `watod` using `$ watod -h`, or see [docs/dev/watod.md](../dev/watod.md).

We recommend you add `~/wato_monorepo_v2/` to your `$PATH` variable so that you can run `watod` from anywhere on your computer, not just in your `~/wato_monorepo_v2` directory and add `watod-completion.bash` to your bashrc to enable tab autocomplete:
```bash
echo "export PATH=\$PATH:\$HOME/wato_monorepo_v2/" >> ~/.bashrc
echo "source ~/wato_monorepo_v2/scripts/watod-completion.bash" >> ~/.bashrc
source ~/.bashrc
```
It also has the added bonus of letting you type `watod` instead of `./watod` :)

## If it's your first time
Check out our [Monorepo Samples](../../src/samples/)! These nodes provide a comprehensive understanding of the Monorepo's infrastructure, and they are also a great reference when coding up your own nodes in the Monorepo.

You should also check out the rest of our documentation [here](../../docs/).