## ----------------------- watod Configuration File Override ----------------------------

############################ ACTIVE MODULE CONFIGURATION ############################
## List of active modules to run, defined in docker-compose.yaml.
##
## List of active modules to run, defined in docker-compose.yaml.
## Possible values:
##   - infrastructure     	:   starts visualization tools (foxglove and/or vnc and/or data_stream)
##	 - perception			:	starts perception nodes
##	 - world_modeling		:	starts world modeling nodes
##	 - action				:	starts action nodes
##	 - simulation			:	starts simulation
##   - samples             	:   starts sample ROS2 pubsub nodes

ACTIVE_MODULES="infrastructure perception"

############################## OPTIONAL CONFIGURATIONS ##############################
## Name to append to docker containers. DEFAULT = "<your_watcloud_username>"
COMPOSE_PROJECT_NAME="lereljic"
FOXGLOVE_BRIDGE_PORT="8771"

## Tag to use. Images are formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced with dashes.
## DEFAULT = "<your_current_github_branch>"
TAG="latest"

# Docker Registry to pull/push images. DEFAULT = "ghcr.io/watonomous/wato_monorepo"
# REGISTRY_URL=""
