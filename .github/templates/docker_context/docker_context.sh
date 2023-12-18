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
REPOSITORY=$(echo "$REGISTRY_URL" | sed 's|^.*/\(.*\)$|\1|')

echo "registry=$REGISTRY" >> $GITHUB_OUTPUT
echo "repository=$REPOSITORY" >> $GITHUB_OUTPUT
