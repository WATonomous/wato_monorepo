#!/bin/bash
set -e

################# Setup watod Environment #################
bash scripts/watod-setup-env.sh

################# Sweep for Docker Services and Profiles #################
# Scans for services and modules in the wato_monorepo,
# dynamically builds a json matrix for downstream CI build and testing
echo "Hi"
# Find docker compose files in 'modules' directory
modules=$(find modules -name "docker-compose*")
echo "Hi"
# Initialize an empty array for JSON objects
json_objects=()
echo "Hi"
# Loop through each profile
while read -r module; do
    # Retrieve docker compose service names
    services=$(docker-compose -f "$module" config --services)
    module_out=$(echo "$module" | sed -n 's/modules\/docker-compose\.\(.*\)\.yaml/\1/p')
    echo "Hi"

    # Loop through each service
    while read -r service_out; do
        # Construct JSON object for each service with profile and service name
        json_object=$(jq -nc --arg module_out "$module_out" --arg service_out "$service_out" \
        '{module: $module_out, service: $service_out}')
        echo "Hi"
        # Append JSON object to the array
        json_objects+=("$json_object")
    done <<< "$services"
done <<< "$modules"

echo "bruh"
# Convert the array of JSON objects to a single JSON array
json_services=$(jq -s . <<< "${json_objects[*]}")
echo "sdfa"
echo "docker_matrix=$json_services" >> $GITHUB_OUTPUT
echo "docker matrix"

################# Setup Docker Registry and Repository Name #################
# Docker Registry to pull/push images
REGISTRY_URL="ghcr.io/watonomous/wato_monorepo"
echo "dwhereee"
REGISTRY=$(echo "$REGISTRY_URL" | sed 's|^\(.*\)/.*$|\1|')
echo "kms"
REPOSITORY=$(echo "$REGISTRY_URL" | sed 's|^.*/\(.*\)$|\1|')
echo "HUH?"
echo "registry=$REGISTRY" >> $GITHUB_OUTPUT
echo "HUH?"
echo "repository=$REPOSITORY" >> $GITHUB_OUTPUT
echo "idk"
