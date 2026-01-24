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
      COMPREPLY=($(compgen -W "${profiles}" -- "${cur}"))
      return 0
      ;;
    bag)
      COMPREPLY=($(compgen -W "${bag_cmds}" -- "${cur}"))
      return 0
      ;;
    -m|--module)
      COMPREPLY=($(compgen -W "${modules}" -- "${cur}"))
      return 0
      ;;
  esac

  # Default: complete with commands
  COMPREPLY=($(compgen -W "${commands}" -- "${cur}"))
  return 0
}

complete -o default -F _watod_completions watod
