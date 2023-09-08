## ----------------------- Watod2 Configuration File Override ----------------------------
## ACTIVE PROFILES CONFIGURATION
## List of active profiles to run, defined in docker-compose.yaml.
##
## Possible values:
##   - vis_tools     		  :   starts visualization tools (vnc and foxglove)
##   - production    		  :   configs for all containers required in production
##   - samples             :   starts sample ROS2 pubsub nodes

# ACTIVE_PROFILES=""


## Name to append to docker containers. DEFAULT = <your_watcloud_username>

# COMPOSE_PROJECT_NAME=""


## Tag to use. Images are formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced with dashes.
## DEFAULT = <your_current_github_branch> 

# TAG=""
#!/bin/bash
from dev_config.sh

# ACTIVE_PROFILES="data_stream"
# ACTIVE_PROFILES="data_stream lidar_publisher lidar_object_detection"
# ACTIVE_PROFILES="multimodal_object_detection"
ACTIVE_PROFILES="data_stream sensor_fusion_publisher camera_detection"
# ACTIVE_PROFILES="data_stream sensor_fusion_publisher lidar_object_detection"

FOXGLOVE_BRIDGE_PORT=8770
