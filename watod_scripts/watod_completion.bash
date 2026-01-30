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
# Copyright (c) 2025-present WATonomous. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Bash completion for watod
# Installed automatically via 'watod install'

_watod_completions() {
  local cur="${COMP_WORDS[COMP_CWORD]}"
  local prev="${COMP_WORDS[COMP_CWORD-1]}"

  # Top-level commands and options
  local commands="up down ps logs build pull install test bag run -m -t -v -h"

  # Recording profiles for 'watod run'
  local profiles="all_sensors camera_only lidar_only --list -l"

  # Bag subcommands
  local bag_cmds="record play info ls convert reindex"

  # Modules
  local modules="all action interfacing perception world_modeling simulation"

  case "${prev}" in
    run)
      mapfile -t COMPREPLY < <(compgen -W "${profiles}" -- "${cur}")
      return 0
      ;;
    bag)
      mapfile -t COMPREPLY < <(compgen -W "${bag_cmds}" -- "${cur}")
      return 0
      ;;
    -m|--module)
      mapfile -t COMPREPLY < <(compgen -W "${modules}" -- "${cur}")
      return 0
      ;;
  esac

  # Default: complete with commands
  mapfile -t COMPREPLY < <(compgen -W "${commands}" -- "${cur}")
  return 0
}

complete -o default -F _watod_completions watod
