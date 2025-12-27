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
# Determine which modules to process
if [[ -n ${MODIFIED_MODULES:-} ]]; then
  modules_to_process="$MODIFIED_MODULES"
else
  # Default: all modules except simulation
  modules_to_process="infrastructure interfacing perception world_modeling action"
fi

$DEBUG && { echo "▶ Modules to process:"; printf '  %s\n' "$modules_to_process"; }

# Build matrix with one entry per module (watod will build entire module)
declare -a json_rows
for module in $modules_to_process; do
  # Skip simulation
  [[ $module == simulation ]] && continue

  $DEBUG && echo "↳   $module" >&2

  json_rows+=("$(jq -nc --arg module "$module" '{module:$module}')")
done

# Build final matrix JSON
matrix=$(printf '%s\n' "${json_rows[@]}" | jq -s '{include: .}' | jq -c .)
# --------------------------- emit outputs ---------------------------
echo "================ EMITTING OUTPUTS ================"
emit "docker_matrix=$matrix"
emit "registry=$REGISTRY"
emit "repository=$REPOSITORY"

# --------------------------- debug report ---------------------------
if $DEBUG; then
  echo -e "\n================ DEBUG REPORT ================"
  jq <<<"$matrix"
  echo -e "\nModules to build: ${#json_rows[@]}"
  echo "=============================================="
fi
