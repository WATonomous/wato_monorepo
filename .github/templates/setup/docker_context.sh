#!/bin/bash
set -e

bash dev_config.sh
#
# profiles=$(find profiles -name "docker-compose*")
# dockerfile_paths=()
# services=()
# while read -r profile; do
#   profile_services=$(docker-compose -f "$profile" config --services)
#
#   while read -r service; do
#     # Parse yaml for docker-compose service and retrieve dockerfile path
#     service_config=$(docker-compose -f "$profile" config | yq ".services.$service")
#     dockerfile_path=$(echo "$service_config" | yq '.build.dockerfile')
#
#     dockerfile_paths+=("$dockerfile_path")
#     services+=("$service")
#   done <<< "$profile_services"
# done <<< "$profiles"
#
# # Combine dockerfile paths into array of json dicts
# # e.g. [{path: ...}, {path: ...}, ...]
# json_paths=$(jq -n '$ARGS.positional' --args -- "${dockerfile_paths[@]}" | \
#   jq -c 'map({path: .})')
# # Combine docker-compose services into array of json dicts
# # e.g. [{service: samples}, {service: vis_tools}, ...]
# json_services=$(jq -n '$ARGS.positional' --args -- "${services[@]}" | \
#   jq -c 'map({service: .})')
# # Zip previous arrays of json dicts for dockerfile path and services into one list
# # e.g. [{path: ..., service: samples}, {path: ..., service: vis_tools}, ...]
# json_context=$(jq -nc \
#   --argjson paths "$json_paths" \
#   --argjson services "$json_services" \
#   '[$paths, $services] | transpose | map(add)')
# echo "context=$json_context" >> "$GITHUB_OUTPUT"

profiles=$(find profiles -name "docker-compose*")
echo "$profiles"

readarray -t services < <(echo "$profiles" | \
  xargs -I{} docker-compose -f {} config --services)
echo "${services[@]}"
json_services=$(jq -nc '$ARGS.positional' --args -- "${services[@]}")
echo "$json_services"
echo "services=$json_services" >> "$GITHUB_OUTPUT"

# Retrieve path of all docker-compose files
# Remove profiles/docker-compose. and .yaml to get string of profiles
# e.g. "samples vis_tools data_stream"
profiles_string=$(echo "$profiles" | \
  sed -e "s/^profiles\/docker-compose\.//" -e "s/\.yaml$//" | \
  paste -sd " ")
echo "profiles=$profiles_string" >> "$GITHUB_OUTPUT"

