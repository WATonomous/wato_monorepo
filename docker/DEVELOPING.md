# Docker Implementation Architecture

Implementation details for the monorepo's Docker build system.

If you are not familiar with docker and docker compose, please consult their documentation:
- https://docs.docker.com/get-started/
- https://docs.docker.com/compose/

That being said, you do not need to know the entirety of docker to make sense of what is going on. Generally what docker does is allow you to setup isolated, reproducible environments in a declarative way. Docker Compose lets you orchestrate and configure how these environments build and start up.

## Directory Structure

**`base/`** - Dockerfiles for building base images (ROS2 + WATonomous tooling). Ran with GitHub Actions

**`config/`** - Configuration files copied into containers (entrypoint script, bashrc, RMW configs)

**`<module>.Dockerfile`** - Module-specific source and dependencies

## Two-Tier Multi-Stage Build System

### Tier 1: Module Dockerfiles
Each module Dockerfile (e.g., `perception.Dockerfile`, `interfacing.Dockerfile`) defines:

**`source` stage**:
- Copies ROS2 packages into `${AMENT_WS}/src`
- Provides source code for dependency scanning
- Optimized for caching: git repos copied before local source

**`dependencies` stage**:
- Installs non-rosdep dependencies
- Used as fallback when packages not available via rosdep

### Tier 2: Template Dockerfile
`template.Dockerfile` consumes module images and creates a standard pipeline:

```dockerfile
ARG MODULE_SOURCE=<module>:source
ARG MODULE_DEPS=<module>:deps
```

Four sequential stages:

1. **`rosdep_install`**: Scans source, installs rosdep dependencies, merges custom deps
2. **`build`**: Runs `colcon build`, installs to `${WATONOMOUS_INSTALL}`
3. **`deploy`**: Removes source, production-ready (target for `docker-compose.yaml`)
4. **`develop`**: Extends `rosdep_install`, adds dev tools, mounts source at runtime (target for `docker-compose.dev.yaml`)

## Docker Compose Structure

Multiple compose files layered together in `modules/` directory:

**`docker-compose.yaml`** - Base service definitions
- Three services per module: `<module>_source`, `<module>_deps`, `<module>_bringup`
- Infrastructure services: Zenoh router, Foxglove, log viewer
- Profile-based activation

**`docker-compose.dev.yaml`** - Development overrides
- Extends base services via `extends` directive
- Changes target to `develop`, command to `sleep infinity`
- Mounts source volumes from `${MONO_DIR}/src/<module>`
- Uses `<module>_dev` profiles

**`docker-compose.dep.yaml`** - Production deployment overrides

**`docker-compose.test.yaml`** - Test configuration overrides

## Build Cache Strategy

At the core of designing the docker infrastructure, was build caching. The monorepo attempts to maximize the usage of caching based on the volatility of certain parts of the stack (ie. source code changes more often than dependencies).

More information about layer caching can be found [here](https://docs.docker.com/build/cache/).
