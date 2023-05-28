# WATonomous Monorepo v2

Dockerized ROS2 setup for the WATonomous Autonomous Driving Software Pipeline

- [WATonomous Monorepo](#watonomous-monorepo)
  - [Getting Started](#getting-started)
  - [Description of Files](#description-of-files)
  - [Documentation](#documentation)
    - [Setup Docs](#setup-docs)
    - [Important Topics for Developers](#important-topics-for-developers)
    - [Monorepo Info](#monorepo-info)
    - [Technical Specification](#technical-specification)
    - [FAQ](#faq)

## Getting Started
**READ THIS**: [docs/setup/setup.md](docs/setup/setup.md)

## Description of Files

- `watod2`. 
  - This is the main bash script that you will use to interface with your containers. More info on `watod2`: [docs/dev/watod2.md](docs/dev/watod2.md).
- `dev_config.sh`. 
  - dev_config.sh will create a [.env file](https://docs.docker.com/compose/env-file/) specifying environment variables for docker-compose. `watod2` automatically runs this script before running any commands. To override variables in `dev_config.sh`, create a `dev_config.local.sh` file and populate it with variables, for example `ACTIVE_PROFILES="perception path_planning"`. `dev_config.sh` will then take this file into account when building the `.env` file.
- `scripts/watod2-completion.bash`.
  - Bash autocomplete for watod2. Adapted from docker-compose. Add `source <MONO_DIR>/scripts/watod2-completion.bash` to your bashrc to use autocomplete.
- `profiles/`: 
  - This folder contains all docker-compose files specifying the services we will run. They are divided into profiles. The root-level `docker-compose.yaml` file is loaded first, then profiles are loaded after. Note that by default no profiles are enabled. To select additional profiles, overwrite `ACTIVE_PROFILES=x` in `dev_config.local.sh`. See the [docker-compose wiki](https://docs.docker.com/compose/extends/). More info on profiles: [docs/dev/profiles.md](docs/dev/profiles.md).
- `docker/`: 
  - This folder contains the `Dockerfiles` for each of our images. [Docker wiki](https://docs.docker.com/engine/reference/builder/).
- `src/`: 
  - Here is where our actual code goes. The folders in `src` will be mounted to the docker images, so changes in the `src` directory will be reflected in the containers. 

## Documentation

### Setup Docs
[docs/setup](docs/setup)

### Important Topics for Developers
[docs/dev](docs/dev)

### Monorepo Info
[docs/monorepo.md](docs/monorepo.md)

### Technical Specification 
Docker, Continuous Integration: [tech_spec.md](docs/tech_spec.md)

### FAQ
[docs/faq.md](docs/faq.md)