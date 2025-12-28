# Developer Guidelines for the Eve Monorepo

If you have not already, you can get a better understanding of working with Docker + ROS2 monorepo's with our onboarding assignment.

Developing in the Eve Monorepo is very similar, except there are a few caveats.

## watod

`watod` is a wrapper script around `docker compose` that manages the monorepo's development environment. It automatically handles environment variables, compose files, and service configuration.

### Installation (Optional)
To run watod from anywhere in your computer, you can install `watod` by running:

```bash
./watod install
```

Otherwise, you can run `watod` as just `./watod` when you are under the `wato_monorepo/` directory.

### Usage

For current usage and all available options:

```bash
watod --help
```

**Important Features:**
- **Service management**: Start, stop, and manage Docker containers (`watod up`, `watod down`, `watod ps`)
- **Terminal access**: Open shells in running containers with `-t` flag (`watod -t perception_bringup_dev`)
  - Note: for full DevContainer experience, run `watod -t` on a container that is denoted as a `DevContainer`. More information about DevContainers [here](#devcontainers)
- **Testing**: Run colcon tests across $ACTIVE_MODULES with (`watod test`)
- **ROS bag management**: Record and play ROS2 bags with automatic mounting (`watod bag <command>`)

To begin using `watod`, edit the [`watod-config.sh`](./watod-config.sh) or create a copy called `watod-config.local.sh` to configure important settings for `watod` like $ACTIVE_MODULES which defines which docker services to start.

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

## DevContainers
The WATonomous monorepo allows for quick, isolated development using docker containers. This means that you can write and test code without any explicit installations on your own machine except for docker.

To setup a DevContainer:
1. Append `:dev` to any of the $ACTIVE_MODULES you specify in `watod-config.sh`.
1. Up containers with `watod up`. Containers denoted as `(DevContainer)` are development environments that you can connect to.
1. Connect to the DevContainer in one of the following ways:

    1. **Terminal Access** Access a devcontainer through your terminal with `watod -t $DEVCONTAINER_NAME`. Replace `$DEVCONTAINER_NAME` with the name of the container that was denoted as `(DevContainer)` when you ran `watod up`
    1. **VSCode Access** Install the DevContainer extension on vscode. Then do `Ctrl+Shift+P` and select `Dev Containers: Attach to Running Container...`. Then select the container prepended with `_dev` as the container you want to connect VSCode into.

1. Once you are inside a DevContainer, make sure you work in `~/ament_ws/`. All `src` files relevant to the module will be mounted from the monorepo to `~/ament_ws/src`. This is a default ROS2 workspace where you can run commands like `colcon`
1. Manage git changes outside of the devcontainer.

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

To run tests for all `$ACTIVE_MODULES`, run

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
To facilitate efficient interprocess communication, we utilize the [Zenoh middleware (rmw_zenoh)](https://github.com/ros2/rmw_zenoh/tree/rolling) with [shared memory support](https://zenoh.io/docs/manual/abstractions/#shared-memory).

What this means is, in the context of ROS2 messages, topics are discovered and routed through Zenoh's peer-to-peer protocol, with large messages automatically passed through shared memory for zero-copy data transfer between processes on the same machine.

## ROS2 Intraprocess Communication
While Zenoh's shared memory support provides efficient zero-copy message passing, we can achieve even faster communication by using [rclcpp_components](https://github.com/ros2/rclcpp/tree/master/rclcpp_components) to form component containers where multiple nodes share the same process.

**Pros**:
- Message passing is just passing a pointer (no serialization, no data copying at all)
- Super fast message passing
**Cons**:
- All nodes are in one process
- If one node fails, the whole process fails

Only group your nodes into a component container if you believe that they are tightly coupled and should fail/start as one.

## Adding Dependencies
**Dependency** any codebase, library, package, that your code depends on.

Dependencies are managed inside a Dockerfile through a variety of tools. When adding external libraries to the wato_monorepo, there are number of ways to do so. The order of methods from best to worst is as follows:
1. **Installing through ROSdep** ROSdep is a dependency manager from ROS which automatically finds compatible libraries to install for a specific version of ROS. Underthehood, ROSdep uses apt for C++ packages and Pip for python packages. ROSdep dependencies are set inside a ros package's `package.xml`. A `package.xml` has three types of fields:
   - `<exec_depend>dependency_name</exec_depend>` is used for when your package only needs the dependency at runtime (not during build)
   - `<test_depend>dependency_name</test_depend>` is used for when your package only needs the dependency at test time
   - `<build_depend>dependency_name</build_depend>` is used for when your package only needs the dependency at build time
   - `<depend>dependency_name</depend>` is used for when you package needs the dependency as a whole
   - This is dangerous as it causes packages to bloat with unneeded dependencies

    To check if the library you want can be installed through ROSdep, you can check [rosdistro](https://github.com/ros/rosdistro). Best way is to clone that repo and Ctrl+F for the package you desire.

1. **Direct `apt` or `pip` installation (not suggested)** You can directly install dependencies in the Dockerfile's `dependencies` stage. This is not recommended as we could run into versioning issues and dockerfile bloat in the future. Also, to make your package open source, you have to make it work with ROSdep. That will heavily increase the odds of your package actually being taken seriously by companies, individuals, research labs, etc.

    **Contribute to opensource! (suggested)** When there doesn't exist a ROSdep key for a given `apt` or `pip` package. Congratulations! You found a quick and easy way to contribute to opensource. To do so:

    1. Fork the https://github.com/ros/rosdistro repository
    2. Add your package to the appropriate YAML file:
       - Pip packages → rosdep/python.yaml
       - System packages → rosdep/base.yaml
    3. Format for pip packages (in python.yaml):

       ```yaml
       python3-yourpackage-pip:
         debian:
           pip:
             packages: [yourpackage]
         fedora:
           pip:
             packages: [yourpackage]
         ubuntu:
           pip:
             packages: [yourpackage]
       ```

```
4. Submit a Pull Request with:
   - Links to package listings (PyPI for pip packages,
  Ubuntu/Debian/Fedora repos for system packages)
   - Brief description of the package and your use case
   - Ensure alphabetical ordering
   - Remove trailing whitespace
5. Requirements:
   - Must be in official repos (PyPI main index for pip, official
  distro repos for apt)
   - Requires review from 2 people before merging
   - Typically merged within a week (you can install the dependency as a direct `apt` or `pip` while you wait)
```

1. **Vendor Package** If the codebase that you want to depend on is not released as a pip or apt dependency, and they ask you to build the codebase from source, then you can create a vendor package of that codebase. To do so, create a package called `<package_name>_vendor`, and then use CMakeLists.txt to build the package using colcon build.

   - **Create CMakeLists.txt**

       ```cmake
         cmake_minimum_required(VERSION 3.8)
         project(package_name_vendor)

         find_package(ament_cmake REQUIRED)

         # Option to force vendor build even if system version exists
         option(FORCE_BUILD_VENDOR_PKG "Build from source instead of using
         system package" OFF)

         # Try to find system-installed version first
         if(NOT FORCE_BUILD_VENDOR_PKG)
           find_package(package_name QUIET)
         endif()

         if(package_name_FOUND)
           message(STATUS "Found system package_name, skipping build")
           ament_package()
           return()
         endif()

         # Build from source using ExternalProject
         include(ExternalProject)

         ExternalProject_Add(package_name_external
           GIT_REPOSITORY https://github.com/owner/repo.git
           GIT_TAG v1.0.0  # Specific version/tag/commit

           CMAKE_ARGS
             -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
             -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
             # Add other cmake options as needed
             -DBUILD_TESTING=OFF
             -DBUILD_EXAMPLES=OFF

           # Patch if needed
           # PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_SOURCE_DIR}/patches/fix.patch
         )

         # Install marker file so other packages know this was built
         install(FILES
           ${CMAKE_CURRENT_BINARY_DIR}/package_name_external-prefix/src/package_name_external-stamp/package_name_external-build
           DESTINATION share/${PROJECT_NAME}
         )

         ament_package()
       ```

   - **Alternative: Download and Build Archive**

       For non-git sources:

       ```cmake
       ExternalProject_Add(package_name_external
         URL https://example.com/package-1.0.0.tar.gz
         URL_HASH SHA256=abc123...

         CMAKE_ARGS
           -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
           -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
       )
       ```

   - **For Non-CMake Projects**

       If the source uses Make, Autotools, or custom build:

       ```cmake
       ExternalProject_Add(package_name_external
         GIT_REPOSITORY https://github.com/owner/repo.git
         GIT_TAG v1.0.0

         CONFIGURE_COMMAND ./configure --prefix=${CMAKE_INSTALL_PREFIX}
         BUILD_COMMAND make -j$(nproc)
         INSTALL_COMMAND make install

         BUILD_IN_SOURCE 1
       )
       ```

    **Contribute to Opensource!** Now that you've create a vendor package, you can release it to the ROS buildfarm and become a co-maintainer of the package! Refer to the below steps (skip to Option B) to see how.

1. **Clone the repo into the dockerfile** This is only for repos can can be built with colcon, BUT do not release themselves as part of the ROS build farm (cannot be downloaded through rosdep).

    **Contribute to Opensource!** If the package can be built with colcon and its dependencies are already handled by rosdep, then you have an opportunity to become a co-maintainer of that package! To do so, do the following:

    > We highly encourage this because it not only helps boosts WATonomous' reputation, but also yours in the opensource community. You also get to say that you are a co-maintainer of a package that could be really important (ie. SLAM, Bytetrack, etc.)

    1. Contact the Original Authors First:

         ```
         Open an issue or discussion:
         Title: "Interest in releasing this package to ROS build farm"

         Hi! I'd like to use this package in production and would love to see
         it
         available via apt. Would you be open to:
         1. Me helping release it to the jazzy distribution?
        2. Becoming a co-maintainer to handle releases?

        I'm happy to do the work with bloom and submit the PR.
        ```

    2. Wait for Response

        Best case is they say yes
       - You coordinate with them
       - They give you push access to their repo (or a -release repo)
       - You become a co-maintainer

        No response after ~2 weeks: You can proceed independently (see below)

    3. Independent Release

        If they don't respond or aren't interested in maintaining:

        Option A: Release from a fork

        **Fork their repo to the WATonomous GitHub then release from the fork**

        > **TODO (eddy)** Currently, we are trying to figure out how to do the release process for our monorepo. If you run into this, let my know

        bloom-release --rosdistro jazzy --track jazzy package_name \
          --github-org your_username

        In the rosdistro PR, explain:
        This is a release of [original_repo] maintained by [original_author].

       - Original repo: https://github.com/original/repo
       - I've reached out to the maintainer (link to issue)
       - No response after 2 weeks / Maintainer is no longer active
       - I'm taking on maintenance responsibility for ROS releases

        Option B: **Create a vendor package**

        your_org/their_package_vendor

        This signals you're maintaining a vendored version.

    4. Maintenance Responsibility

        By releasing, WATonomous (or you) is committing to:
       - Respond to build farm issues
       - Update for new ROS distros
       - Fix critical bugs (or at least coordinate fixes)
        Add watonomous yourself to package.xml:

        ```xml
          <maintainer email="hello@watonomous.com">WATonomous</maintainer>
          <author email="original@email.com">Original Author</author>
        ```
