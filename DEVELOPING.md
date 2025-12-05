# Developer Guidelines for the Eve Monorepo

If you have not already, you can get a better understanding of working with Docker + ROS2 monorepo's with our onboarding assignment.

Developing in the Eve Monorepo is very similar, except there are a few caveats.

## Base Images and Docker Registry

WATonomous hosts a docker registry where we store various docker images on the internet. We currently use ghcr.io

```bash
# BEFORE YOU RUN, you need to create a personal access token with write:packages
docker login ghcr.io
# Username: <your github username>
# Password: <your personal access token>
```

Theses base images are used in our dockerfiles to provide a starting point for all of our docker images.

ALL WATONOMOUS BASE IMAGES ARE BUILT USING THE WATO_MONOREPO. It is configured [here](.github/include/base_image_config.json), and must be ran manually in GitHub CI (search for the job **Build Monorepo Base Images** under the **Actions** tab). Note, only administrators can trigger new image builds.

### How are the wato_monorepo base images created?

All wato_monorepo base images are created using [GitHub Workflows](https://docs.github.com/en/actions/using-workflows/about-workflows). GitHub Workflows are automated procedures defined in a GitHub repository that can be triggered by various GitHub events, such as a push or a pull request, to perform tasks like building, testing, and deploying code.

## Pre-commit

Pre-commit is used to handle all of our code formatting and linting

```bash
sudo apt-get install -y --no-install-recommends libxml2-utils # for xml linting
pip install pre‑commit  # if you haven't installed it already
pre-commit install
pre-commit run --all-files
```

## Testing

### Unittesting with wato_test
All nodes must contain some set of unittests. These can either be testing a node's functions individually, or testing the complete node through IPC in a deterministic way.

We use catch2 tests to do our unittesting. To make the testing process easier to setup. We've introduced a helper library used to test all nodes in the monorepo.

#### To Use
[wato_test](src/wato_test/) contains a basic CMAKE macro to let you setup a test with the appropriate libraries. It also contains helper nodes to help you test publishers, subscribers, servers, and clients in an event-driven way. Use this package whenever you are setting up tests.

To run tests for all $ACTIVE_MODULES, run

```bash
./watod test
```

### Playing ROS2 Bags

A bag is a file that stores serialized ROS2 message data. We can play a bag to make data available on various topics.
More on bags can be found here: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html.

#### To Use
[**Download rosbags for local development here!**](https://drive.google.com/drive/folders/127Kw3o7Org474rkK1wwMDRZKiYHaR0Db)

We expose `ros2 bag` CLI through watod with some hardcoded constraints (mcap bag format and 20GB bag splitting).

To view all possible commands, run the following:

```bash
$ watod bag --help
usage: ros2 bag [-h]
                Call `ros2 bag <command> -h` for more detailed usage. ...

Various rosbag related sub-commands

options:
  -h, --help            show this help message and exit

Commands:
  convert   Given an input bag, write out a new bag with different settings
  info      Print information about a bag to the screen
  list      Print information about available plugins to the screen
  play      Play back ROS data from a bag
  record    Record ROS data to a bag
  reindex   Reconstruct metadata file for a bag
  to_video  Convert a ROS 2 bag into a video

  Call `ros2 bag <command> -h` for more detailed usage.
```

`watod bag` manages rosbags located in `$BAG_DIRECTORY` which is a settable directory in `watod-config.sh` and defaults to `$MONO_DIR/bags`. It will create a `$BAG_DIRECTORY/bags/` directory if it doesn't exist.

Some common commands:

```bash
watod bag ls # special command to list all bags inside the $BAG_DIRECTORY
watod bag record -a -o $BAG_NAME # records all topics, saves in $BAG_DIRECTORY/$BAG_NAME
watod bag play $BAG_NAME # plays the bag located in $BAG_DIRECTORY/$BAG_NAME
watod bag convert --help # we can do processing on the bags using ros2 bag conversions
```

If you are using WATcloud, play-only bags exist in:

```bash
/mnt/wato-drive2/nuscenes_mcap/ros2bags # Nuscenes Data converted into ROSbag format
/mnt/wato-drive2/rosbags2 # Old rosbag recordings
```

## Simulation

Eve uses Carla Simulator for offline tests. See [CARLA_README.md](src/simulation/CARLA_README.md).

## Documentation Guidelines

We follow a by-package mentality for documentation. Every package in the monorepo should contain:

1. `README.md` to discuss general usage
1. `DEVELOPING.md` to discuss technical specifications and general development patterns of that package

Generally you should:
- Avoid potential **documentation rot** by reducing documentation that directly refers to implementation details.
  - **Why?** The implementation could change, and you forget to change the documentation associated with it.
- Assume users of your package knows ROS2, and also generally how to use the monorepo infrastructure
- Provide enough information so that anyone knows how hit the ground running.
- Always keep track of current hacks

We also have the global README.md and DEVELOPING.md as shown to help you navigate around.

## Logging and Viewing Data

`watod up` will startup both a Foxglove Bridge and a Log Viewer.

### Foxglove Bridge

Use the port number to connect to the Foxglove Bridge via the Foxglove Website / App

### Log Viewer

Click the link to bringup a page containing all the logs of each of the containers

> If you are running `watod` on a remote machine, you will have to forward the respective ports to your local machine.

# General Implementation Details

## IPC Middleware
To facilitate efficient interprocess communication, we utilize the [CycloneDDS middleware](https://github.com/ros2/rmw_cyclonedds) with [Iceoryx shared memory message passing underthehood](https://cyclonedds.io/docs/cyclonedds/latest/shared_memory/shared_mem_config.html).

What this means is, in the context of ROS2 messages, topics are discovered over the network (DDS), but actual data is passed through shared memory.

## ROS2 Intraprocess Communication
The CycloneDDS Middleware with Shared Memory can still be inefficient at times. So if we want to guarentee the fastest form of message transfer, we can use [rclcpp_components](https://github.com/ros2/rclcpp/tree/master/rclcpp_components) to form component containers where multiple nodes share the same process.

**Pros**:
- Message passing is just passing a pointer (no serialization, no data copying at all)
- Super fast message passing
**Cons**:
- All nodes are in one process
- If one node fails, the whole process fails

Only group your nodes into a component container if you believe that they are tightly coupled and should fail/start as one.
