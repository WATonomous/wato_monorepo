#!/bin/bash
set -e

################# Sweep for Docker Services and Modules #################
# Scans for services and modules in the wato_monorepo,
# dynamically builds a json matrix for downstream CI build and testing

# Find docker compose files in 'modules' directory
modules=$(find modules -maxdepth 1 -name "docker-compose*")

# Initialize an empty array for JSON objects
json_objects=()

# Check for infrastructure changes
TEST_ALL=false
if [[ $MODIFIED_MODULES = "infrastructure" ]]; then
    TEST_ALL=true
fi

# Loop through each module
while read -r module; do

    # Retrieve docker compose service names
    services=$(docker compose -f "$module" config --services)
    module_out=$(echo "$module" | sed -n 's/modules\/docker-compose\.\(.*\)\.yaml/\1/p')

    # Skip simulation module
    if [[ 'simulation' = $module_out ]]; then
        continue
    fi

    # Only work with modules that are modified
    if [[ $MODIFIED_MODULES != *$module_out* && $TEST_ALL = "false" ]]; then
        continue
    fi

    # Loop through each service
    while read -r service_out; do
        # Temporarily skip perception services that have too large image size
        if  [[ "$service_out" == "lane_detection" ]] || \
            [[ "$service_out" == "camera_object_detection" ]] || \
            [[ "$service_out" == "semantic_segmentation" ]]; then
            continue
        fi
        # Construct JSON object for each service with module and service name
        json_object=$(jq -nc --arg module_out "$module_out" --arg service_out "$service_out" \
        '{module: $module_out, service: $service_out}')
        # Append JSON object to the array
        json_objects+=($json_object)
    done <<< "$services"
done <<< "$modules"

# Convert the array of JSON objects to a single JSON array
json_services=$(jq -nc '[( $ARGS.positional[] | fromjson )]' --args -- ${json_objects[*]})
echo "docker_matrix=$(echo $json_services | jq -c '{include: .}')" >> $GITHUB_OUTPUT

################# Setup Docker Registry and Repository Name #################
# Docker Registry to pull/push images
REGISTRY_URL="ghcr.io/watonomous/wato_monorepo"

REGISTRY=$(echo "$REGISTRY_URL" | sed 's|^\(.*\)/.*$|\1|')

echo "registry=$REGISTRY" >> $GITHUB_OUTPUT
