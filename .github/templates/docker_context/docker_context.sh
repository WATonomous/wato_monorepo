#!/usr/bin/env bash
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
set -euo pipefail

################################################################################
# docker_context.sh — simple & reliable
# -------------------------------------
# Builds a GitHub‑Actions matrix of **unique Dockerfiles** referenced by
# docker‑compose files in `modules/`.  Loosely based on the original “works but
# no dockerfile” script the user shared.
#
# Differences from the original:
#   • Includes a `dockerfile` field resolved from each service’s build stanza.
#   • De‑duplicates identical Dockerfiles (first service wins).
#   • Still obeys MODIFIED_MODULES and skips heavy perception services.
#
# Usage
#   MODIFIED_MODULES="perception action" ./docker_context.sh --debug
################################################################################

DEBUG=false
for arg in "$@"; do
  case $arg in --debug|-d) DEBUG=true ;; esac
done

############################# helper for outputs ###############################
emit() {
  echo "$1"
  if [[ -n ${GITHUB_OUTPUT:-} ]]; then
    echo "$1" >> "$GITHUB_OUTPUT"
  fi
}

################################# constants ####################################
REGISTRY_URL="ghcr.io/watonomous/wato_monorepo"
REGISTRY="${REGISTRY_URL%%/*}"
REPOSITORY="${REGISTRY_URL#*/}"

################################ build matrix ##################################
shopt -s lastpipe
compose_file="modules/docker-compose.yaml"

$DEBUG && echo "▶ Using compose file: $compose_file"

declare -A seen
declare -a json_rows

# Services to skip
skip_services="log_viewer carla_sim carla_ros_bridge carla_viz carla_sample_node carla_notebooks"

# Determine which modules to process
if [[ -n ${MODIFIED_MODULES:-} ]]; then
  modules_to_process="$MODIFIED_MODULES"
else
  # Default: all modules except simulation
  modules_to_process="infrastructure interfacing perception world_modeling action"
fi

$DEBUG && { echo "▶ Modules to process:"; printf '  %s\n' $modules_to_process; }

for module in $modules_to_process; do
  # Skip simulation
  [[ $module == simulation ]] && continue

  # Always include infrastructure profile as other services depend on it
  cfg=$(docker compose -f "$compose_file" --profile infrastructure --profile "$module" config --format json)

  echo "$cfg" | jq -r '
    .services | to_entries[] | select(.value.build?) | [ .key,
      (.value.build.context // "."),
      (if (.value.build|type)=="object" then (.value.build.dockerfile // "Dockerfile") else "Dockerfile" end),
      (.value.profiles[0] // "")
    ] | @tsv' | while IFS=$'\t' read -r svc ctx df_rel svc_profile; do

      # Skip services in skip list
      if [[ " $skip_services " =~ " $svc " ]]; then
        continue
      fi

      # Only process services that belong to the current module profile
      if [[ "$svc_profile" != "$module" ]]; then
        continue
      fi

      [[ $ctx = /* ]] && ctx_abs="$ctx" || ctx_abs="$(realpath -m "$(dirname "$compose_file")/$ctx")"
      [[ $df_rel = /* ]] && df_abs="$df_rel" || df_abs="$(realpath -m "$ctx_abs/$df_rel")"
      df_repo_rel="$(realpath --relative-to=. "$df_abs")"

      $DEBUG && echo "↳   $module/$svc → $df_repo_rel" >&2

      [[ -n ${seen[$df_repo_rel]:-} ]] && continue
      seen[$df_repo_rel]=1
      json_rows+=("$(jq -nc --arg module "$module" --arg service "$svc" --arg dockerfile "$df_repo_rel" '{module:$module,service:$service,dockerfile:$dockerfile}')")
  done

done

# Build final matrix JSON – every row piped into jq -s to form an array
matrix=$(printf '%s
' "${json_rows[@]}" | jq -s '{include: .}' | jq -c .)
# --------------------------- emit outputs ---------------------------
echo "================ EMITTING OUTPUTS ================"
emit "docker_matrix=$matrix"
emit "registry=$REGISTRY"
emit "repository=$REPOSITORY"

# --------------------------- debug report ---------------------------
if $DEBUG; then
  echo -e "
================ DEBUG REPORT ================
"
  jq <<<"$matrix"
  echo -e "
Unique Dockerfiles scheduled: ${#json_rows[@]}"
  echo   "----------------------------------------------"
  printf '%s
' "${!seen[@]}" | sort
  echo "=============================================="
fi
