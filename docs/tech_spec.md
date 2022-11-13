
# Technical Specification

## Docker

Except in exceptional situations, each dockerfile in `docker/` should have three stages: `base`, `repo`, and `debug`. `base` contains the runtime dependencies for the project the image will be running. `repo` copies the source code into the image. `debug` contains all the fancy tools we use during development. For example, our code-server and vnc server. When we run the containers in production, we have no need for code-servers or vnc-servers, so we use the `repo` stage. In development, we use the `debug` stage. The target stage to launch can be specified in `dev_config.sh`

## Continuous Integration

Continuous integration is handled configured in the `.gitlab-ci.yml` configuration file. There, we have a list of the various jobs we want our continuous integration servers to run. For example, jobs using the `build_template` will build and upload new docker images to our container registry. Additional scripts called by `.gitlab-ci.yml` are stored in `.gitlab/ci/`. Eventually, we want to expand this configuration to include automated unit testing and integration testing. See https://docs.gitlab.com/ee/ci/ for more information.

Jobs are executed and managed by "Runners". We have runners enabled on `wato-wato3.uwaterloo.ca` and `wato-nuc.uwaterloo.ca`. The runner on `wato-wato3.uwaterloo.ca` uses Kubernetes to manage jobs using the [Docker in Docker (DIND) workflow](https://docs.gitlab.com/ee/ci/docker/using_docker_build.html#use-the-docker-executor-with-the-docker-image-docker-in-docker). With this workflow, we build and run our continuous integration jobs inside of docker 
containers, allowing us to parallelize jobs for higher efficiency. Using DIND, we also know that our jobs will be isolated and reproducable.

## Making a new docker image

More documentation TBD. For now, see Charles's Docker Workshop: https://drive.google.com/file/d/1F78lINGzhCQlVni-znqyyePRVttKpeju/view?usp=sharing 
