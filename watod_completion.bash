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
# bash completion for watod
# Provides autocomplete for watod command options, commands, and arguments

__watod_previous_extglob_setting=$(shopt -p extglob)
shopt -s extglob

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

# Available modules (matches get_all_modules() in watod)
__watod_modules() {
  echo "action interfacing perception world_modeling simulation all"
}

# Get running services using docker compose
__watod_running_services() {
  local mono_dir
  mono_dir=$(__watod_get_mono_dir)
  if [[ -f "$mono_dir/modules/docker-compose.yaml" ]]; then
    docker compose -f "$mono_dir/modules/docker-compose.yaml" ps --services --filter status=running 2>/dev/null
  fi
}

# Get all defined services
__watod_all_services() {
  local mono_dir
  mono_dir=$(__watod_get_mono_dir)
  if [[ -f "$mono_dir/modules/docker-compose.yaml" ]]; then
    docker compose -f "$mono_dir/modules/docker-compose.yaml" ps --services 2>/dev/null
  fi
}

# Get available bags (directories containing metadata.yaml)
__watod_list_bags() {
  local mono_dir
  mono_dir=$(__watod_get_mono_dir)
  # Check for bag directory in config or use default
  local bag_dir="$mono_dir/bags"
  if [[ -f "$mono_dir/watod-config.local.sh" ]]; then
    local config_bag_dir
    config_bag_dir=$(grep "export BAG_DIRECTORY=" "$mono_dir/watod-config.local.sh" | cut -d'"' -f2)
    [[ -n "$config_bag_dir" ]] && bag_dir="$config_bag_dir"
  fi

  if [[ -d "$bag_dir" ]]; then
    find "$bag_dir" -name "metadata.yaml" -type f 2>/dev/null | while read -r line; do
      local b_dir
      b_dir=$(dirname "$line")
      echo "${b_dir#"$bag_dir"/}"
    done
  fi
}

# ros2 bag subcommands
__watod_bag_subcommands() {
  echo "record play info list reindex convert ls"
}

# docker compose commands that watod passes through
__watod_compose_commands() {
  echo "up down ps logs exec run build pull push restart stop start kill rm images config top events"
}

# Complete module names with optional :dev suffix
__watod_complete_modules() {
  local cur="$1"
  local modules
  modules=$(__watod_modules)

  # If cur ends with :, complete with dev
  if [[ "$cur" == *: ]]; then
    local base="${cur%:}"
    mapfile -t COMPREPLY < <(compgen -W "${base}:dev" -- "$cur")
  else
    # Add :dev variants
    local all_modules="$modules"
    for m in $modules; do
      all_modules="$all_modules ${m}:dev"
    done
    mapfile -t COMPREPLY < <(compgen -W "$all_modules" -- "$cur")
  fi
}

# Complete running services only (for -t/--terminal)
__watod_complete_running_services() {
  local cur="$1"
  local services
  services=$(__watod_running_services)
  mapfile -t COMPREPLY < <(compgen -W "$services" -- "$cur")
}

# Complete all services (for test command)
__watod_complete_all_services() {
  local cur="$1"
  local services
  services=$(__watod_all_services)
  mapfile -t COMPREPLY < <(compgen -W "$services" -- "$cur")
}

_watod() {
  local cur prev words cword
  _get_comp_words_by_ref -n : cur prev words cword

  # watod-specific options
  local watod_short_opts="-v -m -t -h"
  local watod_long_opts="--verbose --module --terminal --help"
  local watod_all_opts="$watod_short_opts $watod_long_opts"

  # watod commands
  local watod_commands="install test bag"

  # All top-level commands
  local all_commands
  all_commands="$watod_commands $(__watod_compose_commands)"


  COMPREPLY=()

  # Handle option arguments
  case "$prev" in
    -m|--module)
      __watod_complete_modules "$cur"
      return 0
      ;;
    -t|--terminal)
      __watod_complete_running_services "$cur"
      return 0
      ;;
  esac

  # Determine if we're completing after a command
  local command=""
  local command_pos=0
  for ((i=1; i < cword; i++)); do
    case "${words[$i]}" in
      -m|--module|-t|--terminal) ((i++)) ;;
      -v|--verbose|-h|--help) ;;
      -*) ;;
      *)
        command="${words[$i]}"
        command_pos=$i
        break
        ;;
    esac
  done

  # Complete based on context
  if [[ -z "$command" ]]; then
    if [[ "$cur" == -* ]]; then
      if [[ "$cur" == --* ]]; then
        mapfile -t COMPREPLY < <(compgen -W "$watod_long_opts" -- "$cur")
      else
        mapfile -t COMPREPLY < <(compgen -W "$watod_all_opts" -- "$cur")
      fi
    else
      mapfile -t COMPREPLY < <(compgen -W "$all_commands" -- "$cur")
    fi
  else
    case "$command" in
      test)
        __watod_complete_all_services "$cur"
        ;;
      bag)
        if [[ $((command_pos + 1)) -eq $cword ]]; then
          mapfile -t COMPREPLY < <(compgen -W "$(__watod_bag_subcommands)" -- "$cur")
        elif [[ $((command_pos + 2)) -eq $cword ]]; then
          local subcmd="${words[$((command_pos + 1))]}"
          if [[ "$subcmd" == @(play|info|reindex|convert) ]]; then
            mapfile -t COMPREPLY < <(compgen -W "$(__watod_list_bags)" -- "$cur")
          fi
        fi
        ;;
      up)
        mapfile -t COMPREPLY < <(compgen -W "-d --build --force-recreate --no-deps --no-build" -- "$cur")
        ;;
      logs)
        mapfile -t COMPREPLY < <(compgen -W "-f --tail --timestamps" -- "$cur")
        __watod_complete_all_services "$cur"
        ;;
      ps)
        mapfile -t COMPREPLY < <(compgen -W "--services --filter --format --all" -- "$cur")
        ;;
      exec|run)
        __watod_complete_all_services "$cur"
        ;;
      stop|start|restart|kill|rm|build|pull)
        __watod_complete_all_services "$cur"
        ;;
    esac
  fi

  return 0
}

eval "$__watod_previous_extglob_setting"
unset __watod_previous_extglob_setting

complete -F _watod watod
