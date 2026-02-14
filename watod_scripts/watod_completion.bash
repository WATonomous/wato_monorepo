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
# SPDX-License-Identifier: Apache-2.0
#
# Bash completion for watod
# Provides autocomplete for watod command options, commands, and arguments

# Find monorepo root
__watod_get_mono_dir() {
  local dir="$PWD"
  while [[ "$dir" != "/" ]]; do
    if [[ -f "$dir/watod-config.sh" ]]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  # Fallback to current directory if not found
  echo "$PWD"
}

# Available modules (dynamically fetched from watod)
__watod_get_modules() {
  local watod_cmd="$1"
  local modules
  # Try parsing help output
  modules=$($watod_cmd -h 2>/dev/null | sed -n '/Use "all" for all modules/,/Append :bag/p' | grep -v 'Use "all"' | grep -v 'Append' | awk '{print $1}' | sed 's/://g' | tr '\n' ' ')
  if [[ -z "$modules" ]]; then
    # Fallback to defaults if parsing fails
    modules="action interfacing perception world_modeling simulation"
  fi
  echo "$modules all"
}

# Get all defined services across all modules
__watod_get_services() {
  local watod_cmd="$1"
  local services
  # Use watod config to get services if possible (most accurate as it respects defaults)
  services=$($watod_cmd -m all config --services 2>/dev/null)

  if [[ -z "$services" ]]; then
     # Fallback to docker compose directly if watod fails (e.g. not in monorepo root properly)
     local mono_dir
     mono_dir=$(__watod_get_mono_dir)
     if [[ -f "$mono_dir/modules/docker-compose.yaml" ]]; then
       # We need to try to include other compose files to see all services,
       # but without arguments we can't be sure. Just reading the main one is a safe fallback.
       docker compose -f "$mono_dir/modules/docker-compose.yaml" ps --services 2>/dev/null
     fi
  else
    echo "$services"
  fi
}

# Get running services only
__watod_get_running_services() {
  local watod_cmd="$1"
  local services
  services=$($watod_cmd ps --services --filter status=running 2>/dev/null)

  if [[ -z "$services" ]]; then
     # Fallback
     local mono_dir
     mono_dir=$(__watod_get_mono_dir)
     if [[ -f "$mono_dir/modules/docker-compose.yaml" ]]; then
       docker compose -f "$mono_dir/modules/docker-compose.yaml" ps --services --filter status=running 2>/dev/null
     fi
  else
    echo "$services"
  fi
}

# Get available bags (directories containing metadata.yaml)
__watod_list_bags() {
  local mono_dir
  mono_dir=$(__watod_get_mono_dir)
  local bag_dir="$mono_dir/bags"

  # Check for custom bag directory in config
  if [[ -f "$mono_dir/watod-config.local.sh" ]]; then
    local config_bag_dir
    config_bag_dir=$(grep "export BAG_DIRECTORY=" "$mono_dir/watod-config.local.sh" | cut -d'"' -f2)
    [[ -n "$config_bag_dir" ]] && bag_dir="$config_bag_dir"
  fi

  if [[ -d "$bag_dir" ]]; then
    # Return directory names relative to bag_dir
    find "$bag_dir" -name "metadata.yaml" -type f -printf "%P\n" 2>/dev/null | while read -r line; do
      dirname "$line"
    done | grep -v "^\.$" | sort -u
  fi
}

_watod_completions() {
  local cur="${COMP_WORDS[COMP_CWORD]}"
  local prev="${COMP_WORDS[COMP_CWORD-1]}"
  local words=("${COMP_WORDS[@]}")
  local cword=$COMP_CWORD

  # Find the watod executable
  local watod_cmd="watod"
  if ! command -v watod &> /dev/null; then
    if [[ -f "./watod" ]]; then
      watod_cmd="./watod"
    fi
  fi

  # Top-level commands and options
  local commands="up down ps logs build pull install test bag run -m -t -v -s -h"
  local options="-v --verbose -m --module -t --terminal -s --setup-dev-env -h --help"

  # 1. Handle Options contexts regardless of where we are
  case "$prev" in
    -m|--module)
      local modules
      modules=$(__watod_get_modules "$watod_cmd")
      local suggestions=""
      for m in $modules; do
        if [[ "$m" == "all" ]]; then
          suggestions+="all all:dev "
        else
          suggestions+="${m} ${m}:dev ${m}:bag "
        fi
      done
      mapfile -t COMPREPLY < <(compgen -W "$suggestions" -- "$cur")
      return 0
      ;;
    -s|--setup-dev-env)
      local modules
      modules=$(__watod_get_modules "$watod_cmd")
      mapfile -t COMPREPLY < <(compgen -W "$modules" -- "$cur")
      return 0
      ;;
    -t|--terminal)
      local services
      services=$(__watod_get_running_services "$watod_cmd")
      mapfile -t COMPREPLY < <(compgen -W "$services" -- "$cur")
      return 0
      ;;
  esac

  # 2. Determine if we are completing a specific command or if -s is active
  local command=""
  local is_setup_dev_env=false
  for ((i=1; i < cword; i++)); do
    local word="${words[$i]}"
    case "$word" in
      -s|--setup-dev-env)
        is_setup_dev_env=true
        ;;
      -m|--module|-t|--terminal)
        ((i++)) # Skip next argument
        ;;
      -v|--verbose|-h|--help)
        ;;
      -*)
        ;; # Skip other flags
      *)
        command="$word"
        break
        ;;
    esac
  done

  # Special handling for -s/--setup-dev-env which acts like a command taking multiple modules
  if [[ "$is_setup_dev_env" == "true" ]]; then
    local modules
    modules=$(__watod_get_modules "$watod_cmd")
    mapfile -t COMPREPLY < <(compgen -W "$modules" -- "$cur")
    return 0
  fi

  # 3. Handle Command Logic
  if [[ -z "$command" ]]; then
    # We are at top level
    if [[ "$cur" == -* ]]; then
      # Suggest options
      mapfile -t COMPREPLY < <(compgen -W "$options" -- "$cur")
    else
      # Suggest commands
      mapfile -t COMPREPLY < <(compgen -W "$commands" -- "$cur")
    fi
    return 0
  fi

  # We have a command, complete arguments for it
  case "$command" in
    bag)
      local bag_subcmds="record play info ls convert reindex"

      # Check if we already have a subcommand
      local subcmd=""
      # Find the subcommand index relative to 'bag'
      local bag_idx=-1
      for ((j=1; j < cword; j++)); do
         if [[ "${words[$j]}" == "bag" ]]; then
            bag_idx=$j
            break
         fi
      done

      if [[ $bag_idx -ne -1 && $((bag_idx + 1)) -lt $cword ]]; then
         subcmd="${words[$((bag_idx + 1))]}"
      fi

      if [[ -z "$subcmd" ]]; then
        mapfile -t COMPREPLY < <(compgen -W "$bag_subcmds" -- "$cur")
      else
        # We are completing args for a subcommand
        case "$subcmd" in
           play|info|reindex|convert)
              mapfile -t COMPREPLY < <(compgen -W "$(__watod_list_bags)" -- "$cur")
              ;;
           record)
              if [[ "$cur" == -* ]]; then
                 local record_opts="-o --output -s --storage -b --max-bag-size --compression-mode"
                 mapfile -t COMPREPLY < <(compgen -W "$record_opts" -- "$cur")
              fi
              ;;
        esac
      fi
      ;;

    test)
       if [[ "$prev" == "--services" ]]; then
           mapfile -t COMPREPLY < <(compgen -W "$(__watod_get_services "$watod_cmd")" -- "$cur")
       elif [[ "$prev" == "--packages" ]]; then
           # We don't have an easy way to list packages yet, fallback to file completion?
           # Or maybe nothing.
           true
       elif [[ "$cur" == -* ]]; then
           mapfile -t COMPREPLY < <(compgen -W "--services --packages --pre-profiles --all-profiles" -- "$cur")
       else
           # Default argument to test is service or packge
           mapfile -t COMPREPLY < <(compgen -W "$(__watod_get_services "$watod_cmd")" -- "$cur")
       fi
       ;;

    run)
      # watod run profiles + services (since it might pass through to docker compose run)
      local profiles="all_sensors camera_only lidar_only --list -l"
      local services
      services=$(__watod_get_services "$watod_cmd")
      mapfile -t COMPREPLY < <(compgen -W "$profiles $services" -- "$cur")
      ;;

    exec)
      local services
      services=$(__watod_get_running_services "$watod_cmd")
      mapfile -t COMPREPLY < <(compgen -W "$services" -- "$cur")
      ;;

    up|down|logs|ps|build|pull|restart|stop|start|kill|rm)
      # Common docker compose commands that take services

      if [[ "$command" == "up" && "$cur" == -* ]]; then
          mapfile -t COMPREPLY < <(compgen -W "-d --build --force-recreate --no-deps" -- "$cur")
          return 0
      fi

      if [[ "$command" == "logs" && "$cur" == -* ]]; then
          mapfile -t COMPREPLY < <(compgen -W "-f --tail --timestamps" -- "$cur")
          return 0
      fi

      local services
      services=$(__watod_get_services "$watod_cmd")
      [[ "$command" == "down" ]] && services="all $services"

      mapfile -t COMPREPLY < <(compgen -W "$services" -- "$cur")
      ;;

    *)
      # default file completion for unknown commands
      ;;
  esac

  return 0
}

# Set up the completion
complete -o bashdefault -o default -F _watod_completions watod
