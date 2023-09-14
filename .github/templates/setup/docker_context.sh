#!/bin/bash
set -e

bash scripts/watod-setup-env.sh

profiles=$(find profiles -name "docker-compose*")

# Retrieve docker compose service names in bash array
readarray -t services < <(echo "$profiles" | \
  xargs -I{} docker-compose -f {} config --services)
# Convert bash array to json
json_services=$(jq -nc '$ARGS.positional' --args -- "${services[@]}")
echo "services=$json_services" >> "$GITHUB_OUTPUT"

# Retrieve path of all docker-compose files
# Remove profiles/docker-compose. and .yaml to get string of profiles
# e.g. "samples vis_tools data_stream"
profiles_string=$(echo "$profiles" | \
  sed -e "s/^profiles\/docker-compose\.//" -e "s/\.yaml$//" | \
  paste -sd " ")
echo "profiles=$profiles_string" >> "$GITHUB_OUTPUT"

