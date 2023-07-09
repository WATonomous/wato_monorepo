# CARLA Setup in Monorepo
CARLA is an open-source autonomous driving simulator based on Unreal Engine 4. The primary ways of interacting with the CARLA setup are through the Python API or with ROS2 (via the CARLA ROS Bridge). Both of these methods are explained in greater detail further down in this document. 

The goal of the  CARLA setup is to provide an easy way for WATonomous members to interact with the simulator without having to setup anything themselves. So, if you have any questions or suggestions regarding any part of the CARLA setup (or questions about CARLA in general) please bring them up in Discord in #simulation-general or contact Vishal Jayakumar (masteroooogway on Discord or at [v3jayaku@watonomous.ca](mailto:v3jayaku@watonomous.ca)).

- [CARLA WATonomous Documentation](#using-carla-setup-in-monorepo)
    - [Getting Started](#getting-started)
    - [Initial Setup](#initial-setup) 
    - [Interacting with CARLA using the Python API](#interacting-with-carla-using-the-python-api)
    - [Using a ROS Node to interact with CARLA](#using-a-ros-node-to-interact-with-carla)
    - [CARLA Visualization using Foxglove Studio](#carla-visualization-using-foxglove-studio)
    - [FAQ](#faq)
        - [CARLA is running very slow (approx. 3 fps)](#every-time-i-make-a-new-branch-i-need-to-rebuild-all-the-images-from-scratch)
        - [I get an error saying port has already been allocated](#i-get-an-error-saying-port-has-already-been-allocated)


## Getting Started
**READ THIS**: [docs/setup.md](docs/setup.md)

**READ THIS**: [docs/readme.md](docs/readme.md)

**Make sure you are confortable with navigating the monorepo before reading**

## Initial Setup

To run CARLA and all associated containers first add `carla` as an `ACTIVE_PROFILE` and declare `CARLA_NOTEBOOKS_PORT=[port]` in `dev_config.local.sh`. This will cause the following containers to launch when `watod2 up` is run: `carla_server`, `carla_ros_bridge` and `carla_notebooks` (`carla_notebooks` will launch with an open port specified by `CARLA_NOTEBOOKS_PORT`).

## Interacting with CARLA using the Python API

This is the simplest way of interacting with CARLA and most importantly does not require the usage or knowledge of ROS. The documentation below will only show the setup procedure for using the CARLA Python API in the WATOnomous server (via Jupyter Notebook). The full CARLA Python API documentation can be found here ([CARLA Python API Documentation](https://carla.readthedocs.io/en/0.9.13/python_api/)). 

**On the WATOnomous Server**

Forward the port you specified in the `CARLA_NOTEBOOKS_PORT` variable in `dev_config.local.sh` using either the `ports` section of VS Code or by running the command `ssh -L 8888:localhost:8888 <username>@<machine>-ubuntu1.watocluster.local` on your local machine (replace 8888 with the port you specified in the `CARLA_NOTEBOOKS_PORT` variable).

**On your local machine (your personal laptop/pc)**

Open any web browser (Chrome, Firefox, Edge etc.) and type `localhost:8888` (replace 8888 with the port you specified in the `CARLA_NOTEBOOKS_PORT` variable) and click enter. You should be automatically redirected to the Jupyter Notebook home screen where you should see the file `carla_notebooks.ipynb`. Open that file and follow the instructions in it.

## Using a ROS Node to interact with CARLA

**Ensure that you have a good understanding of ROS and writing ROS nodes before proceeding**

The currently recommended (and the simplest) way of using a ROS Node to interact with CARLA is to simply edit the already setup CARLA sample node (currently only in Python unfortunately). Since the package (and all required CARLA and ROS dependencies) is already fully setup the only file you need to edit is the `carla_sample_node.py` found under `wato_monorepo_v2/src/simulation/carla_sample_node/carla_sample_node/`. Since the container is setup with a volume, simply restart the containers with `watod2 down` followed by `watod2 up` (or any other method to restart the containers) and the node should be rebuilt and launched with the changes you made to `carla_sample_node.py`. The sample node currently shows how to publish a message to enable (and keep enabled) autopilot, subsribe to the GNSS sensor topic and log the sensor data to a file.
 
## CARLA Visualization using Foxglove Studio

Foxglove Studio is a tool to visualize and interact with ROS messages. Using Foxglove Studio, data such as Camera, LiDAR, Radar etc. can be visualized. Foxglove Studio also allows for messages to be published to topics, for example to enable autopilot or set vehicle speed. The documentation below will only show the setup procedure for using Foxglove Studio with the WATOnomous server. Further documentation regarding how to use Foxglove Studio and all its features can be found here ([Foxglove Studio Documentation](https://foxglove.dev/docs/studio))

**On the WATOnomous Server**

Add `data_stream` as an `ACTIVE_PROFILE` and declare `FOXGLOVE_BRIDGE_PORT=[port]` in `dev_config.local.sh`. This will launch the `foxglove.Dockerfile` container with an open port when `watod2 up` is ran. 

It exposes the port specified by the `FOXGLOVE_BRIDGE_PORT` variable, which you will need to forward to your local machine. This can either be done in the `ports` section of VS Code or by running the command `ssh -L 8765:localhost:8765 <username>@<machine>-ubuntu1.watocluster.local` on your local machine (replace 8765 with the port you specified in the `FOXGLOVE_BRIDGE_PORT` variable).

**On your local machine (your personal laptop/pc)**

Open Foxglove Studio on your local machine using either the desktop app ([Download Here](https://foxglove.dev/studio)) or the [Web Application](https://studio.foxglove.dev/) (only supported on Chrome). Click on Open Connection then replace the default port (8765) with the port you specified in the `FOXGLOVE_BRIDGE_PORT` variable. Then click the Open button and after a few seconds Foxglove Studio should connect. You should now be able to access any of the topic being published or subscribed by the CARLA ROS Bridge.

## FAQ

### CARLA is running very slow (approx. 3 fps)

This is expected. The ROS bridge causes CARLA to render all the sensor data which slows down the simulation considerably. While this may be annoying when viewing real-time camera output or trying to control the car manual, the simulation is still running accurately.

### I get an error saying port has already been allocated

This is due another running container exposing the same port you are trying to expose. Go to `dev_config.local.sh` and change the port that is causing the conflict to another 4 digit number.