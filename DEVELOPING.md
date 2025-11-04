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

[wato_test](src/wato_test/) contains a basic CMAKE macro to let you setup a test with the appropriate libraries. It also contains helper nodes to help you test publishers, subscribers, servers, and clients in an event-driven way. Use this package whenever you are setting up tests.

### Playing ROS2 Bags

A bag is a file that stores serialized ROS2 message data. We can play a bag to make data available on various topics.
More on bags can be found here: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html.

#### To Use

Add `infrastructure` as an `ACTIVE_PROFILE` in `watod-config.sh`.

Run `watod up` (or however you want to launch the `infrastructure` service).

The working directory of the `data_stream` container should have a `nuscenes` directory, which contains the NuScenes dataset converted to ros2bag format. To confirm this, run `watod run data_stream ls nuscenes` to view the available bags. Each bag has its own directory. The location of the `.mcap` file is `<name>/<name>_0.mcap`. For example, one of the bags is in `nuscenes/NuScenes-v1.0-mini-scene-0061/NuScenes-v1.0-mini-scene-0061_0.mcap`.

Now, using `watod run data_stream [ROS2 BAG COMMAND]` you can run any `ros2 bag ...` command as documented here: http://wiki.ros.org/rosbag/Commandline. You probably want to explore `ros2 bag play ...`: http://wiki.ros.org/rosbag/Commandline#rosbag_play. (Since this documentation is in ROS1, you can replace `rosbag` with `ros2 bag` to run the equivalent command in ROS2)

Example: `watod run data_stream ros2 bag play ./nuscenes/NuScenes-v1.0-mini-scene-0061/NuScenes-v1.0-mini-scene-0061_0.mcap`

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
