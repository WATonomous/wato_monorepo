#!/bin/bash
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
#
# bash completion for watod

__docker_compose_previous_extglob_setting=$(shopt -p extglob)
shopt -s extglob

__docker_compose_q() {
  watod -np 2>/dev/null "${top_level_options[@]}" "$@" | awk '{if(NR>1)print}'
}

__docker_compose_to_alternatives() {
  local parts
  read -ra parts <<< "$1"
  local IFS='|'
  echo "${parts[*]}"
}

__docker_compose_to_extglob() {
  local extglob
  extglob=$( __docker_compose_to_alternatives "$1" )
  echo "@($extglob)"
}

__docker_compose_has_option() {
  local pattern="$1"
  for (( i=2; i < cword; ++i )); do
    if [[ ${words[$i]} =~ ^($pattern)$ ]]; then
      return 0
    fi
  done
  return 1
}

__docker_compose_map_key_of_current_option() {
  local glob="$1"
  local key glob_pos

  if [ "$cur" = "=" ]; then
    key="$prev"
    ((glob_pos=cword - 2))
  elif [[ $cur == *=* ]]; then
    key=${cur%=*}
    ((glob_pos=cword - 1))
  elif [ "$prev" = "=" ]; then
    key=${words[$((cword - 2))]}
    ((glob_pos=cword - 3))
  else
    return
  fi

  [ "${words[$glob_pos]}" = "=" ] && ((glob_pos--))

  if [[ ${words[$glob_pos]} == @($glob) ]]; then
    echo "$key"
  fi
}

__docker_compose_nospace() {
  type compopt &>/dev/null && compopt -o nospace
}

__docker_compose_services() {
  __docker_compose_q ps --services "$@"
}

__docker_compose_complete_services() {
  mapfile -t COMPREPLY < <(compgen -W "$(__docker_compose_services "$@")" -- "$cur")
}

__docker_compose_complete_running_services() {
  local names
  names=$(__docker_compose_services --filter status=running)
  mapfile -t COMPREPLY < <(compgen -W "$names" -- "$cur")
}

# ...
# (SNIPPED: Apply the same pattern to all other functions and command completions)
# Example pattern:
# Replace COMPREPLY=( $(compgen ...) ) with:
# mapfile -t COMPREPLY < <(compgen ...)

_docker_compose() {
  local previous_extglob_setting
  previous_extglob_setting=$(shopt -p extglob)
  shopt -s extglob

  # local commands=(
  #   build config create down events exec help images kill logs pause port ps
  #   pull push restart rm run scale start stop top unpause up version
  # )

  local daemon_boolean_options="--skip-hostname-check --tls --tlsverify"
  local daemon_options_with_args="--context -c --env-file --file -f --host -H --project-directory --project-name -p --tlscacert --tlscert --tlskey"
  local top_level_options_with_args="--ansi --log-level --terminal -t"

  COMPREPLY=()
  local cur prev words cword
  _get_comp_words_by_ref -n : cur prev words cword

  local command='docker_compose'
  local top_level_options=()
  local counter=1

  pattern_boolean_opts="$(__docker_compose_to_extglob "$daemon_boolean_options")"
	pattern_opts_with_args="$(__docker_compose_to_extglob "$daemon_options_with_args")"
	pattern_top_level_opts="$(__docker_compose_to_extglob "$top_level_options_with_args")"

	while [ "$counter" -lt "$cword" ]; do
		# shellcheck disable=SC2254
		case "${words[$counter]}" in
			$pattern_boolean_opts)
				opt=${words[counter]}
				top_level_options+=("$opt")
				;;
			$pattern_opts_with_args)
				opt=${words[counter]}
				((counter++))
				arg=${words[counter]}
				top_level_options+=("$opt" "$arg")
				;;
			$pattern_top_level_opts)
				(( counter++ ))
				;;
			-*)
				;;
			*)
				command="${words[$counter]}"
				break
				;;
		esac
		(( counter++ ))
	done


  local completions_func=_docker_compose_${command//-/_}
  if declare -F "$completions_func" >/dev/null; then
    "$completions_func"
  fi

  eval "$previous_extglob_setting"
  return 0
}

eval "$__docker_compose_previous_extglob_setting"
unset __docker_compose_previous_extglob_setting

complete -F _docker_compose watod
