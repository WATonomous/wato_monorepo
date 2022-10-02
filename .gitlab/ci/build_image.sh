#!/bin/bash
set -e

# Build an invidiual docker image 
# ENV VARIABLES:
#    - SERVICE_NAME: Name of the service in docker-compose.yaml. 
#    - TARGETS (OPTIONAL): 
#         List of targets to build. Optional.


export USER=docker
export BRANCH=$CI_COMMIT_REF_NAME
export DOCKER_BUILDKIT=1
bash dev_config.sh

docker login -u watoinfra -p watonomous

if [ -z ${TARGETS+x} ];
then 
  echo "Pulling $CI_REGISTRY_IMAGE/$SERVICE_NAME:$CI_COMMIT_REF_NAME"
  # Pull pre-built image from registry if availble. If not available, pull the develop image.
  if time bash watod -a -v pull $SERVICE_NAME 2>&1 >/dev/null | tee /dev/stderr |
      grep 'Some service image(s) must be built from source by running' &> /dev/null
  then
    echo "Pulling $CI_REGISTRY_IMAGE/$SERVICE_NAME:develop"
    time TAG=develop bash watod -a -v pull $SERVICE_NAME
  fi
  
  echo "Building $CI_REGISTRY_IMAGE/$SERVICE_NAME:$CI_COMMIT_REF_NAME"
  time bash watod -a -v build --build-arg BUILDKIT_INLINE_CACHE=1 $SERVICE_NAME 
    echo "Pushing $CI_REGISTRY_IMAGE/$SERVICE_NAME:$CI_COMMIT_REF_NAME"
  time bash watod -a -v push $SERVICE_NAME
  
else 
  echo "Building Targets: $TARGETS"
  for TARGET in $TARGETS; do
    echo "Pulling $CI_REGISTRY_IMAGE/$SERVICE_NAME:$TARGET-$CI_COMMIT_REF_NAME"
    # Pull pre-built image from registry if availble. If not available, pull the develop image.
    if time bash watod -a -v pull $SERVICE_NAME 2>&1 >/dev/null | tee /dev/stderr |
        grep 'Some service image(s) must be built from source by running' &> /dev/null
    then
      echo "Pulling $CI_REGISTRY_IMAGE/$SERVICE_NAME:$TARGET-develop"
      time TAG=develop bash watod -a -v pull $SERVICE_NAME
    fi
    
    echo "Building $CI_REGISTRY_IMAGE/$SERVICE_NAME:$TARGET-$CI_COMMIT_REF_NAME"
    time TARGET_STAGE=$TARGET bash watod -a -v build --build-arg BUILDKIT_INLINE_CACHE=1 $SERVICE_NAME 
    echo "Pushing $CI_REGISTRY_IMAGE/$SERVICE_NAME:$TARGET-$CI_COMMIT_REF_NAME"
    time TARGET_STAGE=$TARGET bash watod -a -v push $SERVICE_NAME
  done
fi
