
# Technical Specification

## Docker
Except in exceptional situations, each dockerfile in `docker/` should have two stages: `base` and `repo`. `base` contains the runtime dependencies for the project the image will be running. `repo` copies the source code into the image. The target stage to launch can be specified in `watod-config.sh`. In the future, your specific subteam may expand on build stages.

To optimize build times, and make the most out of our Docker registry, please stucture your Dockerfile for optimal cache management. More information on cache mamagement and Dockerfile structure can be found [here](https://docs.docker.com/build/cache/).

## Continuous Integration

Continuous integration is handled configured in the `.github/` configuration folder. There, we have a list of the various jobs we want our continuous integration servers to run.

Jobs are executed and managed by "Runners". We have runners enabled on `wato-wato3.uwaterloo.ca` and `wato-nuc.uwaterloo.ca`. The runner on `wato-wato3.uwaterloo.ca` uses Kubernetes to manage jobs using the [Docker in Docker (DIND) workflow](https://docs.gitlab.com/ee/ci/docker/using_docker_build.html#use-the-docker-executor-with-the-docker-image-docker-in-docker). With this workflow, we build and run our continuous integration jobs inside of docker
containers, allowing us to parallelize jobs for higher efficiency. Using DIND, we also know that our jobs will be isolated and reproducable.

## Making a new docker image

See how to dev [here](dev/how_to_dev.md)
