#!/bin/bash
set -e

################# Sweep for Docker Services and Profiles #################
# Scans for services and modules in the wato_monorepo,
# dynamically builds a json matrix for downstream CI build and testing
source scripts/watod-setup-env.sh

# Find docker compose files in 'modules' directory
modules=$(find modules -name "docker-compose*")

# Initialize an empty array for JSON objects
json_objects=()

# Loop through each profile
while read -r profile; do
    # Retrieve docker compose service names
    services=$(docker-compose -f "$profile" config --services)
    profile=$(echo "$profile" | sed -n 's/modules\/docker-compose\.\(.*\)\.yaml/\1/p')
    # Loop through each service
    while read -r service; do
        # Construct JSON object for each service with profile and service name
        json_object=$(jq -nc --arg profile "$profile" --arg service "$service" \
        '{profile: $profile, service: $service}')
        # Append JSON object to the array
        json_objects+=("$json_object")
    done <<< "$services"
done <<< "$modules"

# Convert the array of JSON objects to a single JSON array
json_services=$(jq -s . <<< "${json_objects[*]}")
echo "docker_matrix=$json_services" >> $GITHUB_OUTPUT

################# Setup Registry Name #################
echo "registry=$REGISTRY" >> $GITHUB_OUTPUT

################# Setup Sub Registry Name #################
echo "repository=$REPOSITORY" >> $GITHUB_OUTPUT
