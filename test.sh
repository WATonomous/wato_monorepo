#!/bin/bash
set -e

################# Sweep for Docker Services and Profiles #################
# Scans for services and modules in the wato_monorepo,
# dynamically builds a json matrix for downstream CI build and testing

# Find docker compose files in 'modules' directory
modules=$(find modules -name "docker-compose*")

# Initialize an empty array for JSON objects
json_objects=()

# Loop through each profile
while read -r module; do
    # Retrieve docker compose service names
    services=$(docker-compose -f "$module" config --services)
    module_out=$(echo "$module" | sed -n 's/modules\/docker-compose\.\(.*\)\.yaml/\1/p')

    # Loop through each service
    while read -r service_out; do
        # Construct JSON object for each service with profile and service name
        json_object=$(jq -nc --arg module_out "$module_out" --arg service_out "$service_out" \
        '{module: $module_out, service: $service_out}')
        # Append JSON object to the array
        json_objects+=("$json_object")
    done <<< "$services"
done <<< "$modules"

# Convert the array of JSON objects to a single JSON array
json_services=$(jq -s . <<< "${json_objects[*]}")
echo "docker_matrix=$json_services"

################# Setup Registry Name #################
echo "registry=$REGISTRY"

################# Setup Sub Registry Name #################
echo "repository=$REPOSITORY"


Perception (Perception Docker Compose)
2D camera object detection (ros2 humble, ubuntu 2204)
Traffic light detection  (ros2 humble, ubuntu 2204)
Traffic sign detection  (ros2 humble, ubuntu 2204)
Semantic segmentation  (ros2 humble, ubuntu 2204)
Lane detection (ros2, ubuntu 2004, tensorrt)
Lidar object detection (ros2, ubuntu 2004, tensorrt, cuda 11.3)
Radar detection (ros2 and ubuntu 2204)
Tracking (tbd, ros2 ubuntu)
World Modeling (World Modeling Docker Compose and Action Docker Compose)
HD map container (base ros2 cpp container with lanelet library)
Occupancy grid interface container (base ros2 cpp container with maybe a voxel library like open3d)
occupancy grid segmentation container (cuda environment with pytorch)
motion forecasting container (cuda environment to run our eventual prediction research code)
localization container (base ros2 cpp w/ gtsam library)
behavior container (cuda environment with pytorch) (Action)
MPC (Action Docker Compose)
control container
Simulation (Simulation Docker Compose)
Simulation container
